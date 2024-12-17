/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 00:25:57
 * @LastEditTime: 2024-12-18 01:54:33
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
    plt::subplot2grid(WINDOW_ROWS, WINDOW_COLS, 0, 0, 3, 1);
    imshow(ego.outlook, ego.cur_state.to_vector());
    imshow(pedestrian.outlook, pedestrian.cur_state.to_vector(),
           pedestrian.true_intent == PedestrianIntention::LEFT);
    plt::xlim(-6, 6);
    plt::ylim(-2.5, 22.5);
    plt::title("Jaywalking Scenario");

    // belief update
    plt::subplot2grid(WINDOW_ROWS, WINDOW_COLS, 0, 1);
    static const std::vector<double> BAR_POS = {0.0, 1.5};
    static const std::vector<std::string> INTENTION_STRING = {"R", "L"};
    plt::barh(BAR_POS, pedestrian.intention_belief, 1.0, {"red", "dodgerblue"});
    plt::yticks(BAR_POS, INTENTION_STRING);
    plt::xlim(0, 1);
    plt::ylim(-0.8, 2.3);
    plt::title("Planning Result");

    // trajectory x-t
    plt::subplot2grid(WINDOW_ROWS, WINDOW_COLS, 1, 1);
    plt::plot({1, 3, 2, 4});

    // trajectory y-t
    plt::subplot2grid(WINDOW_ROWS, WINDOW_COLS, 2, 1);
    plt::plot({1, 3, 2, 4});

    plt::pause(0.001);
}
