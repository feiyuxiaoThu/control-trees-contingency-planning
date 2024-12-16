/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-14 00:46:19
 * @LastEditTime: 2024-12-17 00:50:13
 * @FilePath: /dive-into-contingency-planning/src/contingency_planning.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include "simulator/simulator_jaywalking.hpp"

int main(int argc, char** argv) {
    
    SimulatorJaywalking simulator;
    simulator.pedestrian.cur_state.x = 0;
    simulator.pedestrian.cur_state.y = 12;
    simulator.pedestrian.intent = Intention::LEFT;
    simulator.show_scenario();
    plt::show();

    return 0;
}
