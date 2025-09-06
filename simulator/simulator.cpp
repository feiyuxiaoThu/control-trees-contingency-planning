/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:18:40
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 01:53:20
 * @FilePath: /dive-into-contingency-planning/simulator/simulator.cpp
 */

#include "simulator.hpp"

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

Simulator::Simulator(const std::string& config_file) {
    n_pedestrians_ = 4;
    p_crossing_ = 0.15;
    lane_width_ = 3.5;
    spdlog::set_level(spdlog::level::debug);
}

Simulator::~Simulator() { stop(); }

void Simulator::start() {
    if (running_) {
        return;
    }
    running_ = true;
    sim_thread_ = std::thread(&Simulator::simulation_loop, this);
}

void Simulator::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }
}

void Simulator::set_ego_control_input(const Control& input) {}

State Simulator::get_ego_state() const { return State(); }

void Simulator::simulation_loop() {
    const auto start = std::chrono::steady_clock::now();
    foxglove::WebSocketServerOptions ws_options;
    ws_options.host = "127.0.0.1";
    ws_options.port = 8765;
    auto serverResult = foxglove::WebSocketServer::create(std::move(ws_options));
    if (!serverResult.has_value()) {
        std::cerr << foxglove::strerror(serverResult.error()) << '\n';
        return;
    }
    auto server = std::move(serverResult.value());
    auto loop_runtime_channel =
        foxglove::RawChannel::create("/simulation/runtime_secs", "json").value();
    auto ego_car_channel = foxglove::schemas::SceneUpdateChannel::create("/makers/ego_car").value();
    auto pedestrians_channel = foxglove::schemas::SceneUpdateChannel::create("/makers/pedestrians").value();
    auto transform_channel =
        foxglove::schemas::FrameTransformChannel::create("/transform/map_to_baselink").value();

    PedestrianObserver observer(n_pedestrians_);
    ego_state_ = State{0, 0, 0, 3.0};
    while (running_) {
        // purge old
        for (auto i = 0; i < n_pedestrians_; ++i) {
            auto pedestrian = observer.pedestrian(i);
            if (pedestrian && pedestrian->is_done()) {
                observer.erase(i);
                spdlog::debug("pedestrian {} done", pedestrian->id());
            }
        }
        // recreate new
        for (auto i = 0; i < n_pedestrians_; ++i) {
            if (!observer.pedestrian(i)) {
                auto pedestrian = generate_new_pedestrian(p_crossing_, i, lane_width_, ego_state_);
                observer.replace_pedestrian(i, pedestrian);
                spdlog::debug("car_x: {}", ego_state_.x);
                spdlog::debug("pedestrian {} created start_x: {} start_y: {}", pedestrian->id(),
                              pedestrian->get_start_position().x,
                              pedestrian->get_start_position().y);
                break;
            }
        }
        observer.step(ego_state_);
        auto pedestrian_update = observer.observe_pedestrians(ego_state_, lane_width_);
        const double current_time_sec = TimeUtil::NowSeconds();
        if (last_update_time_ > 0.1) {
            const double dt = current_time_sec - last_update_time_;
            ego_state_.x += ego_state_.velocity * std::cos(ego_state_.yaw) * dt;
            ego_state_.y += ego_state_.velocity * std::sin(ego_state_.yaw) * dt;
            ego_state_.yaw += ego_input_.omega * dt;
            ego_state_.velocity += ego_input_.accel * dt;
        }
        last_update_time_ = current_time_sec;

        const auto ego_car_scene_update = get_ego_scene_update();
        const auto ego_pose = ego_state_.to_pose();
        foxglove::schemas::FrameTransform transform;
        transform.timestamp = TimeUtil::NowTimestamp();
        transform.parent_frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.translation = ego_pose.position;
        transform.rotation = ego_pose.orientation;

        auto dur = std::chrono::steady_clock::now() - start;
        const float elapsed_seconds = std::chrono::duration<float>(dur).count();
        std::string elapsed_msg = "{\"elapsed\": " + std::to_string(elapsed_seconds) + "}";
        loop_runtime_channel.log(reinterpret_cast<const std::byte*>(elapsed_msg.data()), elapsed_msg.size());
        ego_car_channel.log(ego_car_scene_update);
        pedestrians_channel.log(pedestrian_update);
        transform_channel.log(transform);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50Hz
    }
}

foxglove::schemas::SceneUpdate Simulator::get_ego_scene_update() {
    foxglove::schemas::ModelPrimitive ego_model;
    ego_model.url =
        "https://raw.githubusercontent.com/PuYuuu/toy-example-of-iLQR/foxglove/images/materials/"
        "lexus.glb";
    ego_model.pose = ego_state_.to_pose();
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
