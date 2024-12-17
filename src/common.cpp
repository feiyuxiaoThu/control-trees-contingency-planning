/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 23:26:43
 * @LastEditTime: 2024-12-17 23:51:03
 * @FilePath: /dive-into-contingency-planning/src/common.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include <fstream>
#include <sstream>

#include "common.hpp"

static const std::filesystem::path source_file_path(__FILE__);
static const std::filesystem::path materials_path =
    source_file_path.parent_path().parent_path() / "images" / "materials";

static bool imread(std::string filename, Outlook& outlook) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        SPDLOG_ERROR("imread(): open {} failed !", filename);
        return false;
    }

    std::string line;
    getline(file, line);
    if (line != "Convert from PNG") {
        SPDLOG_ERROR("imread(): this format is not supported: {}", filename);
        return false;
    }
    getline(file, line);
    std::istringstream iss(line);
    iss >> outlook.rows >> outlook.cols >> outlook.channels;
    outlook.data.resize(outlook.rows * outlook.cols * outlook.channels);
    int idx = 0;
    while (getline(file, line)) {
        std::istringstream iss(line);
        for (int i = 0; i < outlook.channels; ++i) {
            iss >> outlook.data[idx++];
        }
    }
    file.close();

    return true;
}

Vehicle::Vehicle() {
    std::filesystem::path vehicle_img_path = materials_path / "car_cyan.mat.txt";
    imread(vehicle_img_path, outlook);
    outlook.visual_height = 2.0;
    outlook.visual_width = 4.0;
    cur_state.theta = M_PI_2;
}

Pedestrian::Pedestrian() {
    std::filesystem::path pedestrian_img_path = materials_path / "pedestrian_right.mat.txt";
    imread(pedestrian_img_path, outlook);
    outlook.visual_height = 2.5;
    outlook.visual_width = 2.0;

    // default settings
    true_intent = PedestrianIntention::RIGHT;
    intention_belief.resize(2);
    intention_belief[0] = 0.5;
    intention_belief[1] = 0.5;
}
