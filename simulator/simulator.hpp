/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:18:40
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 22:56:43
 * @FilePath: /dive-into-contingency-planning/simulator/simulator.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "foxglove/foxglove.hpp"
#include "foxglove/server.hpp"
#include "simulator/pedestrian.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

class Simulator {
  public:
    Simulator(const std::string& config_file);
    ~Simulator();

    void start();  // 启动仿真线程
    void stop();   // 停止仿真线程

    void set_ego_control_input(const Control& input);

    State get_ego_state() const;

    std::vector<std::shared_ptr<const Pedestrian>> get_pedestrians() const;

  private:
    void simulation_loop();  // 50Hz 仿真循环
    void load_config(const std::string& path);
    void update_objects(double dt);  // 状态推进
    foxglove::schemas::SceneUpdate get_ego_scene_update(const foxglove::schemas::Pose& ego_pose);
    foxglove::schemas::SceneUpdate get_lane_scene_update(const foxglove::schemas::Pose& ego_pose);

  private:
    State ego_state_;
    Control ego_control_input_;

    double last_update_time_ = 0.0;
    std::shared_ptr<PedestrianObserver> observer_{nullptr};

    std::thread sim_thread_;
    std::atomic<bool> running_{false};
    mutable std::shared_mutex ego_state_mutex_;
    mutable std::shared_mutex control_input_mutex_;
    std::condition_variable_any cv_;

    uint32_t n_pedestrians_{0};
    double p_crossing_{0.15};
    double lane_width_{3.5};
};
