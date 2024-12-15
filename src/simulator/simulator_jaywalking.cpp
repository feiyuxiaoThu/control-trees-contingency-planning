/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 00:25:57
 * @LastEditTime: 2024-12-16 00:37:13
 * @FilePath: /dive-into-contingency-planning/src/simulator/simulator_jaywalking.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "simulator/simulator_jaywalking.hpp"

SimulatorJaywalking::SimulatorJaywalking(/* args */) {

}

void SimulatorJaywalking::refresh_scenario() {
    
}

void SimulatorJaywalking::show_scenario() {
    plt::clf();
    imshow(ego.outlook, ego.cur_state.to_vector());
    plt::pause(0.001);
}
