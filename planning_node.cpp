/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:23:02
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-13 18:01:20
 * @FilePath: /dive-into-contingency-planning/planning_node.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "planning/stopline_qp_tree.hpp"
#include "simulator/simulator.hpp"

int main() {
    Simulator simulator("config.yaml");
    simulator.start();

    StopLineQPTree planner(5, 4);
    planner.set_desired_speed(10.0);

    while (true) {
        auto ego_state = simulator.get_ego_state();
        auto pedestrians = simulator.get_pedestrians();
        auto [time, cost] = planner.plan(ego_state, pedestrians);
        auto control = planner.get_control();
        auto planning_info = planner.get_debug_result(ego_state);
        simulator.set_ego_control_input(control);
        simulator.update_planning_info(planning_info);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    simulator.stop();

    return 0;
}
