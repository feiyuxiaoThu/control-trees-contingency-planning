/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:23:02
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 14:52:38
 * @FilePath: /dive-into-contingency-planning/planning_node.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved. 
 */

#include "simulator/simulator.hpp"

int main() {
    Simulator simulator("config.yaml");
    simulator.start();

    while (true) {
        // nothing
    }

    return 0;
}
