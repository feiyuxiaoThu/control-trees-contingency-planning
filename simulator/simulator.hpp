/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:18:40
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-06 23:18:22
 * @FilePath: /dive-into-contingency-planning/simulator/simulator.hpp
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "foxglove/foxglove.hpp"
#include "foxglove/server.hpp"
#include "common/common.hpp"
#include "simulator/pedestrian.hpp"


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
    void update_objects(double dt);  // 状态推进
    foxglove::schemas::SceneUpdate get_ego_scene_update();

  private:
    std::unordered_map<std::string, State> objects_;   // 包含ego和npc车
    State ego_state_;                                   // 自车状态
    Control ego_input_;                                // 自车控制输入

    std::string ego_id_ = "ego";     // 自车ID
    std::shared_mutex state_mutex_;  // 保护 objects_ 和 ego_input_
    double last_update_time_ = 0.0;

    std::thread sim_thread_;
    std::atomic<bool> running_{false};
    std::condition_variable_any cv_;

    uint32_t n_pedestrians_{0};
    double p_crossing_{0.15};
    double lane_width_{3.5};
};