/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:23:02
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 23:52:18
 * @FilePath: /dive-into-contingency-planning/planning_node.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "simulator/simulator.hpp"
#include "planning/stopline_qp_tree.hpp"

#include <spdlog/spdlog.h>

int main() {
    spdlog::set_level(spdlog::level::debug);

    Simulator simulator("config.yaml");
    simulator.start();

    StopLineQPTree planner(5, 4);
    planner.set_desired_speed(10.0);

    while (true) {
        auto ego_state = simulator.get_ego_state();
        auto pedestrians = simulator.get_pedestrians();
        auto [time, cost] = planner.plan(ego_state, pedestrians);
        auto control = planner.get_control();
        spdlog::info("time: {:.2f} cost: {:.2f} control: accel {:.2f}", time, cost, control.accel);
        simulator.set_ego_control_input(control);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if (ego_state.x > 150.0) {
        //     break;
        // }
    }

    simulator.stop();

    return 0;
}
