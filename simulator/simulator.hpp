/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:18:40
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-14 17:15:27
 * @FilePath: /dive-into-contingency-planning/simulator/simulator.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/protos/planning_info.pb.h"
#include "foxglove/foxglove.hpp"
#include "foxglove/server.hpp"
#include "simulator/pedestrian.hpp"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

class Simulator {
  public:
    Simulator() = delete;
    Simulator(const YAML::Node& config);
    ~Simulator();

    void start();  // start the simulation thread
    void stop();   // stop the simulation thread

    void set_ego_control_input(const Control& input);

    void update_planning_info(const planning::protos::PlanningInfo& info);

    State get_ego_state() const;

    std::vector<std::shared_ptr<const Pedestrian>> get_pedestrians() const;

  private:
    void simulation_loop(void);
    void update_pedestrians(void);
    void update_ego_state(void);
    bool register_publish_channels(void);
    foxglove::schemas::SceneUpdate get_ego_scene_update(const foxglove::schemas::Pose& ego_pose);
    foxglove::schemas::SceneUpdate get_lane_scene_update(const foxglove::schemas::Pose& ego_pose);
    foxglove::schemas::SceneUpdate get_trajectory_scene_update(void) const;

  private:
    State ego_state_;
    Control ego_control_input_;
    planning::protos::PlanningInfo planning_info_;
    std::shared_ptr<spdlog::logger> logger_ = nullptr;

    double last_ego_update_time_ = 0.0;
    std::shared_ptr<PedestrianObserver> observer_{nullptr};

    std::thread sim_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> planning_info_updated_{false};
    mutable std::shared_mutex ego_state_mutex_;
    mutable std::shared_mutex control_input_mutex_;
    mutable std::shared_mutex planning_info_mutex_;

    uint32_t n_pedestrians_{0};
    double p_crossing_{0.15};
    double lane_width_{3.5};

    std::unique_ptr<foxglove::WebSocketServer> socket_server_;
    std::unique_ptr<foxglove::RawChannel> loop_runtime_channel_;
    std::unique_ptr<foxglove::schemas::SceneUpdateChannel> ego_car_channel_;
    std::unique_ptr<foxglove::schemas::SceneUpdateChannel> pedestrians_channel_;
    std::unique_ptr<foxglove::schemas::SceneUpdateChannel> lane_lines_channel_;
    std::unique_ptr<foxglove::schemas::SceneUpdateChannel> trajectory_channel_;
    std::unique_ptr<foxglove::schemas::FrameTransformChannel> transform_channel_;
    std::unique_ptr<foxglove::RawChannel> planning_info_channel_;
};
