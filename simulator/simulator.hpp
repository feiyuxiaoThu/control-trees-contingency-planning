/**
 * @file simulator.hpp
 * @author puyu <yu.pu@qq.com>
 * @date 2025/8/3 00:18
 * Copyright (c) puyu. All rights reserved.
 */

#pragma once

#include "foxglove/foxglove.hpp"
#include "foxglove/server.hpp"
#include "common/common.hpp"

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

    // 更新自车控制输入（线程安全）
    void set_ego_control_input(const Control& input);

    // 获取自车状态（线程安全）
    State get_ego_state() const;

    // 获取所有他车状态（线程安全）
    std::unordered_map<std::string, State> getOtherVehicleStates();

  private:
    void simulation_loop();  // 50Hz 仿真循环
    void load_config(const std::string& path);
    void update_vehicles(double dt);  // 状态推进
    foxglove::schemas::SceneUpdate get_ego_scene_update();

  private:
    std::unordered_map<std::string, State> vehicles_;  // 包含ego和npc车
    Control ego_input_;                                // 自车控制输入

    std::string ego_id_ = "ego";     // 自车ID
    std::shared_mutex state_mutex_;  // 保护 vehicles_ 和 ego_input_

    std::thread sim_thread_;
    std::atomic<bool> running_{false};
    std::condition_variable_any cv_;
};