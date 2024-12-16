/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 00:25:57
 * @LastEditTime: 2024-12-17 01:13:30
 * @FilePath: /dive-into-contingency-planning/src/simulator/simulator_jaywalking.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "simulator/simulator_jaywalking.hpp"

SimulatorJaywalking::SimulatorJaywalking(/* args */) {}

void SimulatorJaywalking::refresh_scenario() {}

void SimulatorJaywalking::show_scenario() {
    plt::figure_size(600, 575);
    plt::clf();
    plt::suptitle("Contingency Games for Multi-Agent Interaction",
                  {{"fontsize", "12"}, {"fontweight", "bold"}});

    // main scene visualization
    plt::subplot2grid(PLT_ROWS, PLT_COLS, 0, 0, 3, 1);
    imshow(ego.outlook, ego.cur_state.to_vector());
    imshow(pedestrian.outlook, pedestrian.cur_state.to_vector(), pedestrian.intent == Intention::LEFT);
    plt::xlim(-6, 6);
    plt::ylim(-2.5, 22.5);
    plt::title("jaywalking scenario");

    // belief update
    plt::subplot2grid(PLT_ROWS, PLT_COLS, 0, 1);
    plt::plot({1, 3, 2, 4});
    plt::title("planning result");

    // trajectory x-t
    plt::subplot2grid(PLT_ROWS, PLT_COLS, 1, 1);
    plt::plot({1, 3, 2, 4});

    // trajectory y-t
    plt::subplot2grid(PLT_ROWS, PLT_COLS, 2, 1);
    plt::plot({1, 3, 2, 4});

    plt::pause(0.001);
}
