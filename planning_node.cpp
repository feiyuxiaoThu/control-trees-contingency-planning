/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-08-03 00:23:02
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-16 00:04:28
 * @FilePath: /dive-into-contingency-planning/planning_node.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "planning/stopline_qp_tree.hpp"
#include "simulator/simulator.hpp"

#include <getopt.h>
#include <yaml-cpp/yaml.h>

#include <csignal>

int main(int argc, char** argv) {
    int opt;
    const char* optstring = "c:";
    std::string config_file_path;

    while ((opt = getopt(argc, argv, optstring)) != -1) {
        switch (opt) {
            case 'c':
                config_file_path = optarg;
                break;
            default:
                spdlog::info("Usage: {} [-c]", argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    if (config_file_path.empty()) {
        spdlog::info("Usage: {} [-c]", argv[0]);
        exit(EXIT_FAILURE);
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
        return 1;
    }

    std::atomic_bool done = false;
    static std::function<void()> sigint_handler = [&] { done = true; };
    std::signal(SIGINT, [](int) {
        if (sigint_handler) {
            sigint_handler();
        }
    });

    double max_driving_distance = config["max_driving_distance"].as<double>(10000);
    double max_simulation_time = config["max_simulation_time"].as<double>(300);

    Simulator simulator(config["simulator"]);
    simulator.start();

    StopLineQPTree planner(config["planning"]);

    const auto main_thread_start = std::chrono::steady_clock::now();
    auto next_tick = main_thread_start;
    while (!done) {
        auto ego_state = simulator.get_ego_state();
        auto pedestrians = simulator.get_pedestrians();
        auto [time, cost] = planner.plan(ego_state, pedestrians);
        auto control = planner.get_control();
        auto planning_info = planner.get_debug_result(ego_state);
        simulator.set_ego_control_input(control);
        simulator.update_planning_info(planning_info);

        auto duration = std::chrono::steady_clock::now() - main_thread_start;
        const double elapsed_seconds = std::chrono::duration<double>(duration).count();
        if (ego_state.x >= max_driving_distance || elapsed_seconds >= max_simulation_time) {
            spdlog::info("Reached max driving distance or simulation time, exiting...");
            break;
        }

        next_tick += std::chrono::milliseconds(100);
        std::this_thread::sleep_until(next_tick);
    }

    simulator.stop();

    return 0;
}
