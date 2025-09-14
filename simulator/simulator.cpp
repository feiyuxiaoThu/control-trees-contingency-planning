/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:18:40
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-14 17:44:45
 * @FilePath: /dive-into-contingency-planning/simulator/simulator.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "simulator.hpp"

#include <google/protobuf/descriptor.pb.h>
#include <spdlog/sinks/stdout_color_sinks.h>

Simulator::Simulator(const YAML::Node& config) {
    n_pedestrians_ = config["n_pedestrians"].as<uint32_t>(4);
    p_crossing_ = config["p_crossing"].as<double>(0.05);
    lane_width_ = config["lane_width"].as<double>(3.5);
    double init_speed = config["initial_speed"].as<double>(5.0);
    ego_state_ = State{0, 0, 0, init_speed};
    observer_ = std::make_shared<PedestrianObserver>(n_pedestrians_);

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    logger_ = std::make_shared<spdlog::logger>("simulator_logger", console_sink);
    logger_->set_level(spdlog::level::from_str(config["log_level"].as<std::string>("info")));
    logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^\033[1m%l\033[0m%$] [%s:%#] %v");
}

Simulator::~Simulator() { stop(); }

void Simulator::start() {
    if (running_) {
        return;
    }
    running_ = true;
    sim_thread_ = std::thread(&Simulator::simulation_loop, this);
    LOG_INFO(logger_, "Simulator started.");
}

void Simulator::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }
    LOG_INFO(logger_, "Simulator stopped.");
}

void Simulator::set_ego_control_input(const Control& input) {
    std::unique_lock lock(control_input_mutex_);
    ego_control_input_ = input;
}

void Simulator::simulation_loop() {
    if (!register_publish_channels()) {
        LOG_ERROR(logger_, "Failed to register publish channels.");
        return;
    }

    const auto loop_start = std::chrono::steady_clock::now();
    auto next_tick = loop_start;
    while (running_) {
        update_ego_state();
        update_pedestrians();

        const auto pedestrian_update = observer_->observe_pedestrians(ego_state_, lane_width_);
        const auto ego_pose = get_ego_state().to_pose();
        const auto ego_car_scene_update = get_ego_scene_update(ego_pose);
        const auto lane_scene_update = get_lane_scene_update(ego_pose);

        foxglove::schemas::FrameTransform transform;
        transform.timestamp = TimeUtil::NowTimestamp();
        transform.parent_frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.translation = ego_pose.position;
        transform.rotation = ego_pose.orientation;

        auto dur = std::chrono::steady_clock::now() - loop_start;
        const double elapsed_seconds = std::chrono::duration<double>(dur).count();
        std::string elapsed_msg = "{\"elapsed\": " + std::to_string(elapsed_seconds) + "}";
        loop_runtime_channel_->log(reinterpret_cast<const std::byte*>(elapsed_msg.data()),
                                   elapsed_msg.size());
        ego_car_channel_->log(ego_car_scene_update);
        pedestrians_channel_->log(pedestrian_update);
        lane_lines_channel_->log(lane_scene_update);
        transform_channel_->log(transform);
        if (planning_info_updated_.load()) {
            std::shared_lock lock(planning_info_mutex_);
            std::string debug_info_data = planning_info_.SerializeAsString();
            planning_info_channel_->log(reinterpret_cast<const std::byte*>(debug_info_data.data()),
                                        debug_info_data.size());
            const auto traj_scene_update = get_trajectory_scene_update();
            trajectory_channel_->log(traj_scene_update);

            planning_info_updated_.store(false);
        }

        next_tick += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(next_tick);
    }
}

bool Simulator::register_publish_channels(void) {
    using foxglove::RawChannel;
    using foxglove::WebSocketServer;
    using foxglove::schemas::FrameTransformChannel;
    using foxglove::schemas::SceneUpdateChannel;

    foxglove::WebSocketServerOptions ws_options;
    ws_options.host = "127.0.0.1";
    ws_options.port = 8765;
    auto server_result = WebSocketServer::create(std::move(ws_options));
    if (!server_result.has_value()) {
        std::cerr << foxglove::strerror(server_result.error()) << '\n';
        return false;
    }
    socket_server_ = std::make_unique<WebSocketServer>(std::move(server_result.value()));

    auto make_channel = [](auto&& result) {
        return std::make_unique<std::decay_t<decltype(result.value())>>(std::move(result.value()));
    };

    loop_runtime_channel_ = make_channel(RawChannel::create("/simulation/runtime_secs", "json"));
    ego_car_channel_ = make_channel(SceneUpdateChannel::create("/makers/ego_car"));
    pedestrians_channel_ = make_channel(SceneUpdateChannel::create("/makers/pedestrians"));
    lane_lines_channel_ = make_channel(SceneUpdateChannel::create("/makers/lane_lines"));
    trajectory_channel_ = make_channel(SceneUpdateChannel::create("/makers/trajectory"));
    transform_channel_ = make_channel(FrameTransformChannel::create("/transform/map_to_baselink"));

    auto descriptor = planning::protos::PlanningInfo::descriptor();
    foxglove::Schema schema;
    schema.encoding = "protobuf";
    schema.name = descriptor->full_name();
    // Create a FileDescriptorSet containing our message descriptor
    google::protobuf::FileDescriptorSet file_descriptor_set;
    const google::protobuf::FileDescriptor* file_descriptor = descriptor->file();
    file_descriptor->CopyTo(file_descriptor_set.add_file());
    std::string serialized_descriptor = file_descriptor_set.SerializeAsString();
    schema.data = reinterpret_cast<const std::byte*>(serialized_descriptor.data());
    schema.data_len = serialized_descriptor.size();
    planning_info_channel_ =
        make_channel(RawChannel::create("/planning_info", "protobuf", std::move(schema)));

    return true;
}

void Simulator::update_ego_state(void) {
    const double current_time_sec = TimeUtil::NowSeconds();
    Control local_ctrl{0., 0.};
    {
        std::shared_lock control_lock(control_input_mutex_);
        local_ctrl = ego_control_input_;
    }
    if (last_ego_update_time_ > 0.1) {
        const double dt = current_time_sec - last_ego_update_time_;
        std::unique_lock lock(ego_state_mutex_);
        ego_state_.x += ego_state_.velocity * std::cos(ego_state_.yaw) * dt;
        ego_state_.y += ego_state_.velocity * std::sin(ego_state_.yaw) * dt;
        ego_state_.yaw += local_ctrl.omega * dt;
        ego_state_.velocity += local_ctrl.accel * dt;
    }
    last_ego_update_time_ = current_time_sec;
}

void Simulator::update_pedestrians(void) {
    // purge old
    for (uint32_t i = 0; i < n_pedestrians_; ++i) {
        auto pedestrian = observer_->pedestrian(i);
        if (pedestrian && pedestrian->is_done()) {
            observer_->erase(i);
            LOG_DEBUG(logger_, "pedestrian {} done", pedestrian->id());
        }
    }
    {
        std::shared_lock lock(ego_state_mutex_);
        // recreate new
        for (uint32_t i = 0; i < n_pedestrians_; ++i) {
            if (!observer_->pedestrian(i)) {
                auto pedestrian = generate_new_pedestrian(p_crossing_, i, lane_width_, ego_state_);
                observer_->replace_pedestrian(i, pedestrian);
                LOG_DEBUG(logger_, "car_x: {:.2f}", ego_state_.x);
                LOG_DEBUG(logger_, "pedestrian {} created start_x: {:.2f} start_y: {:.2f}",
                          pedestrian->id(), pedestrian->get_start_position().x,
                          pedestrian->get_start_position().y);
                break;
            }
        }
        observer_->step(ego_state_);
    }
}

foxglove::schemas::SceneUpdate Simulator::get_ego_scene_update(
    const foxglove::schemas::Pose& ego_pose) {
    foxglove::schemas::ModelPrimitive ego_model;
    ego_model.url =
        "https://raw.githubusercontent.com/PuYuuu/dive-into-contingency-planning/master/assets/"
        "mesh/lexus.glb";
    ego_model.pose = ego_pose;
    ego_model.scale = {1, 1, 1};
    ego_model.media_type = "model/gltf-binary";
    ego_model.override_color = false;

    foxglove::schemas::SceneEntity ego_car_entity;
    ego_car_entity.id = "ego_car";
    ego_car_entity.frame_id = "map";
    ego_car_entity.models.push_back(ego_model);
    ego_car_entity.timestamp = TimeUtil::NowTimestamp();
    ego_car_entity.lifetime = foxglove::schemas::Duration{0, 100000000};

    foxglove::schemas::SceneUpdate ego_car_scene_update;
    ego_car_scene_update.entities.push_back(ego_car_entity);

    return ego_car_scene_update;
}

foxglove::schemas::SceneUpdate Simulator::get_lane_scene_update(
    const foxglove::schemas::Pose& ego_pose) {
    double ego_position_x = ego_pose.position.has_value() ? ego_pose.position->x : 0.0;
    foxglove::schemas::SceneUpdate lane_scene_update;
    foxglove::schemas::SceneEntity lane_entity;
    lane_entity.id = "lane_lines";
    lane_entity.frame_id = "map";
    lane_entity.timestamp = TimeUtil::NowTimestamp();
    lane_entity.lifetime = foxglove::schemas::Duration{0, 100000000};

    foxglove::schemas::Pose lane_pose =
        foxglove::schemas::Pose{.position = foxglove::schemas::Vector3{0.0, 0.0, 0.0},
                                .orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 1.0}};
    foxglove::schemas::Color lane_color = foxglove::schemas::Color{1.0, 1.0, 1.0, 1.0};

    foxglove::schemas::LinePrimitive left_line;
    left_line.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    left_line.pose = lane_pose;
    left_line.thickness = 0.12;
    left_line.scale_invariant = false;
    left_line.color = lane_color;
    foxglove::schemas::Point3 start_point{ego_position_x - 20.0, lane_width_ / 2.0, 0.0};
    foxglove::schemas::Point3 end_point{ego_position_x + 100.0, lane_width_ / 2.0, 0.0};
    left_line.points.push_back(start_point);
    left_line.points.push_back(end_point);
    lane_entity.lines.push_back(left_line);

    foxglove::schemas::LinePrimitive right_line;
    right_line.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    right_line.pose = lane_pose;
    right_line.thickness = 0.12;
    right_line.scale_invariant = false;
    right_line.color = lane_color;
    start_point = foxglove::schemas::Point3{ego_position_x - 20.0, -lane_width_ / 2.0, 0.0};
    end_point = foxglove::schemas::Point3{ego_position_x + 100.0, -lane_width_ / 2.0, 0.0};
    right_line.points.push_back(start_point);
    right_line.points.push_back(end_point);
    lane_entity.lines.push_back(right_line);

    foxglove::schemas::LinePrimitive center_line;
    center_line.type = foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    center_line.pose = lane_pose;
    center_line.thickness = 0.08;
    center_line.scale_invariant = false;
    center_line.color = lane_color;
    for (double x = ego_position_x - 20.0; x < ego_position_x + 100.0; x += 5.0) {
        foxglove::schemas::Point3 point{x, 0.0, 0.0};
        center_line.points.push_back(point);
    }
    lane_entity.lines.push_back(center_line);
    lane_scene_update.entities.push_back(lane_entity);

    return lane_scene_update;
}

foxglove::schemas::SceneUpdate Simulator::get_trajectory_scene_update(void) const {
    foxglove::schemas::SceneUpdate traj_scene_update;
    foxglove::schemas::SceneEntity traj_entity;
    traj_entity.id = "trajectory";
    traj_entity.frame_id = "map";
    traj_entity.timestamp = TimeUtil::NowTimestamp();
    traj_entity.lifetime = foxglove::schemas::Duration{0, 200000000};

    foxglove::schemas::Pose traj_pose =
        foxglove::schemas::Pose{.position = foxglove::schemas::Vector3{0.0, 0.0, 0.0},
                                .orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 1.0}};
    foxglove::schemas::Color traj_color = foxglove::schemas::Color{0.38, 0.8, 1.0, 0.9};

    {
        std::shared_lock lock(planning_info_mutex_);
        size_t max_prob_idx = 0;
        double max_prob = 0.0;
        for (size_t idx = 0; idx < planning_info_.solution_size(); ++idx) {
            if (planning_info_.solution(idx).probability() > max_prob) {
                max_prob = planning_info_.solution(idx).probability();
                max_prob_idx = idx;
            }
        }
        if (planning_info_.solution_size() > 0) {
            const auto& speed_profile = planning_info_.solution(max_prob_idx);
            foxglove::schemas::LinePrimitive traj_line;
            traj_line.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
            traj_line.pose = traj_pose;
            traj_line.thickness = 1.0;
            traj_line.scale_invariant = false;
            traj_line.color = traj_color;
            for (int i = 0; i < speed_profile.points_size(); ++i) {
                const auto& sp = speed_profile.points(i);
                if (sp.t() >= 4.5) {
                    break;
                }
                foxglove::schemas::Point3 point{sp.x(), sp.y(), 0.0};
                traj_line.points.push_back(point);
            }
            traj_entity.lines.push_back(traj_line);
        }
    }
    traj_scene_update.entities.push_back(traj_entity);
    return traj_scene_update;
}

State Simulator::get_ego_state() const {
    std::shared_lock lock(ego_state_mutex_);
    return ego_state_;
}

std::vector<std::shared_ptr<const Pedestrian>> Simulator::get_pedestrians() const {
    if (!observer_) {
        return {};
    }
    auto snapshot = observer_->snapshot_pedestrians();

    return snapshot;
}

void Simulator::update_planning_info(const planning::protos::PlanningInfo& info) {
    std::unique_lock lock(planning_info_mutex_);
    planning_info_.CopyFrom(info);
    planning_info_updated_.store(true);
}
