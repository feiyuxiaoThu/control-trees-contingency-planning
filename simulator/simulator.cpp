/**
 * @file simulator.cpp
 * @author puyu <yu.pu@qq.com>
 * @date 2025/8/3 00:18
 * Copyright (c) puyu. All rights reserved.
 */

#include "simulator.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>

Simulator::Simulator(const std::string& config_file) {}

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
    auto ego_car_scene_update = get_ego_scene_update();

    while (running_) {
        auto dur = std::chrono::steady_clock::now() - start;
        const float elapsed_seconds = std::chrono::duration<float>(dur).count();
        std::string elapsed_msg = "{\"elapsed\": " + std::to_string(elapsed_seconds) + "}";
        loop_runtime_channel.log(reinterpret_cast<const std::byte*>(elapsed_msg.data()), elapsed_msg.size());
        ego_car_channel.log(ego_car_scene_update);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50Hz
    }
}

foxglove::schemas::SceneUpdate Simulator::get_ego_scene_update() {
    foxglove::schemas::ModelPrimitive ego_model;
    ego_model.url =
        "https://raw.githubusercontent.com/PuYuuu/dive-into-contingency-planning/develop/assets/"
        "mesh/lexus.glb";
    ego_model.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{0, 0, 0},
                                             foxglove::schemas::Quaternion{0, 0, 0, 1}};
    ego_model.scale = {1, 1, 1};
    ego_model.media_type = "model/gltf-binary";
    ego_model.override_color = false;

    foxglove::schemas::SceneEntity ego_car_entity;
    ego_car_entity.id = "ego_car";
    ego_car_entity.frame_id = "base_link";
    ego_car_entity.models.push_back(ego_model);

    foxglove::schemas::SceneUpdate ego_car_scene_update;
    ego_car_scene_update.entities.push_back(ego_car_entity);

    return ego_car_scene_update;
}
