/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-06 19:25:02
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 16:57:05
 * @FilePath: /dive-into-contingency-planning/simulator/pedestrian.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved. 
 */

#include "simulator/pedestrian.hpp"

#include <spdlog/spdlog.h>
#include <random>

static double rand_01() {
    static thread_local std::mt19937 generator(std::random_device{}());
    static thread_local std::uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(generator);
}

static double draw_p(double median_p) {
    double exponent = std::log(median_p) / std::log(0.5);
    return std::pow(rand_01(), exponent);
}

static bool draw_bool(double average_p) { return (rand_01() < average_p); }

// obstacle creation
std::shared_ptr<Pedestrian> generate_new_pedestrian(double p_crossing, uint32_t id,
                                                    double lane_width, const State& car_position) {
    std::shared_ptr<Pedestrian> pedestrian;

    const double distance_ahead = 20;
    const double distance = distance_ahead + rand_01() * 50.0;
    const double new_x = car_position.x + distance;
    const double new_y = rand_01() > 0.5 ? 0.5 * lane_width + 1 : -0.5 * lane_width - 1;
    const double certainty_x = new_x - 20;  // uncertainty vanishes 15 m to the pedestrian

    const double p = draw_p(p_crossing);
    const double certainty_distance = 10 + (distance_ahead - 5) * rand_01();  // * rand_01();

    bool is_forward = rand_01() > 0.5;
    State init_state{new_x, new_y, is_forward ? 0 : M_PI, 1.0};

    if (draw_bool(p_crossing)) {
        pedestrian = std::shared_ptr<Pedestrian>(
            new CrossingPedestrian(id, init_state, p, certainty_x, is_forward));
    } else {
        pedestrian = std::shared_ptr<Pedestrian>(
            new NonCrossingPedestrian(id, init_state, p, certainty_x, is_forward));
    }

    return pedestrian;
}

bool CrossingPedestrian::is_crossing(double time, double x) {
    if (state_ == PedestrianState::UNCERTAIN) {
        if (x >= crossing_x_) {
            state_ = PedestrianState::MOVING;
            crossing_time_ = time;
            last_update_time_ = time;
            return true;
        }
    }

    if (state_ == PedestrianState::MOVING) {
        if (time < crossing_time_ + crossing_duration_) {
            current_state_.yaw = start_position_.y > 0 ? -M_PI_2 : M_PI_2;
            return true;  // crossing
        } else {
            state_ = PedestrianState::DONE;
            return false;  // crossing finished
        }
    }

    return false;
}

double CrossingPedestrian::get_crossing_probability() const {
    if (state_ == PedestrianState::UNCERTAIN) {
        return p_;
    } else if (state_ == PedestrianState::MOVING) {
        return 1.0;  // crossing
    } else {
        return 1.0;  // crossing finished
    }
}

void CrossingPedestrian::step(double now_s, double now_x) {
    if (is_crossing(now_s, now_x)) {
        const double dt = now_s - last_update_time_;
        current_state_.x += current_state_.velocity * std::cos(current_state_.yaw) * dt;
        current_state_.y += current_state_.velocity * std::sin(current_state_.yaw) * dt;
        last_update_time_ = now_s;
    }
}

bool NonCrossingPedestrian::is_non_crossing(double time, double x) {
    if (state_ == PedestrianState::UNCERTAIN) {
        if (x >= non_crossing_x_) {
            state_ = PedestrianState::MOVING;
            non_crossing_time_ = time;
            last_update_time_ = time;
            return true;
        }
    }

    if (state_ == PedestrianState::MOVING) {
        if (time < non_crossing_time_ + non_crossing_duration_ && x < get_position().x) {
            return true;  // non crossing
        } else {
            state_ = PedestrianState::DONE;
            return false;  // non crossing finished
        }
    }

    return false;
}

double NonCrossingPedestrian::get_crossing_probability() const {
    if (state_ == PedestrianState::UNCERTAIN) {
        return p_;
    } else if (state_ == PedestrianState::MOVING) {
        return 0.0;  // non crossing
    } else {
        return 0.0;  // non crossing finished
    }
}

void NonCrossingPedestrian::step(double now_s, double now_x) {
    if (is_non_crossing(now_s, now_x)) {
        const double dt = now_s - last_update_time_;
        current_state_.x += current_state_.velocity * std::cos(current_state_.yaw) * dt;
        current_state_.y += current_state_.velocity * std::sin(current_state_.yaw) * dt;
        last_update_time_ = now_s;
    }
}

void PedestrianObserver::step(const State& car_position) {
    // take shared lock while iterating and calling step on each pedestrian
    std::shared_lock lock(mutex_);
    for (const auto& pedestrian : pedestrians_) {
        if (pedestrian) {
            pedestrian->step(TimeUtil::NowSeconds(), car_position.x);
        }
    }
}

foxglove::schemas::SceneUpdate PedestrianObserver::observe_pedestrians(const State& car_position,
                                                                       double lane_width) const {
    std::vector<foxglove::schemas::SceneEntity> pedestrian_entities;

    std::shared_lock lock(mutex_);
    for (auto i = 0; i < pedestrians_.size(); ++i) {
        auto pedestrian = pedestrians_[i];

        if (pedestrian) {
            const auto position = pedestrian->get_position();
            const auto start_position = pedestrian->get_start_position();
            double crossing_probability = pedestrian->get_crossing_probability();

            foxglove::schemas::SceneEntity entity;
            entity.id = "pedestrian_" + std::to_string(i);
            entity.frame_id = "map";
            entity.timestamp = TimeUtil::NowTimestamp();
            entity.lifetime = foxglove::schemas::Duration{0, 100000000};

            // cube
            foxglove::schemas::CubePrimitive cube_maker;
            cube_maker.size = foxglove::schemas::Vector3{0.5, 0.5, 2.0};
            cube_maker.color = foxglove::schemas::Color{0.0, 0.0, 0.0, 0.9};
            cube_maker.pose = foxglove::schemas::Pose{
                .position = foxglove::schemas::Vector3{position.x, position.y, 1.0},
                .orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 1.0}};
            if (crossing_probability >= 0.99) {
                cube_maker.color->r = 1.0;
                cube_maker.color->g = 0.0;
                cube_maker.color->b = 0.0;
                const double yaw = (M_PI / 2.0) * (pedestrian->is_forward_direction() ? 1.0 : -1.0);
                cube_maker.pose->orientation = yaw_to_quaternion(yaw);
            } else {
                cube_maker.color->r = 0.0;
                cube_maker.color->g = 1.0;
                cube_maker.color->b = 0.0;

                const double crossing_angle = pedestrian->get_start_position().y > 0 ? 0 : M_PI;
                double final_non_crossing_angle =
                    (M_PI / 2.0) * (pedestrian->is_forward_direction() ? 1.0 : -1.0);
                if (pedestrian->get_start_position().y < 0 && !pedestrian->is_forward_direction()) {
                    // handle particular case otherwise angle interpolation doesn't work
                    final_non_crossing_angle = 3.0 * M_PI / 2.0;
                }
                const double yaw = (1 - crossing_probability) * final_non_crossing_angle +
                                   crossing_probability * crossing_angle;

                cube_maker.pose->orientation = yaw_to_quaternion(yaw);
            }
            if (pedestrian->is_uncertain()) {
                cube_maker.color->r = 1.0;
                cube_maker.color->g = 0.0;
                cube_maker.color->b = 1.0;
            }
            entity.cubes.emplace_back(cube_maker);

            // crossing prediction line
            foxglove::schemas::LinePrimitive crossing_prediction;
            crossing_prediction.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
            crossing_prediction.pose = foxglove::schemas::Pose{
                .position = foxglove::schemas::Vector3{0.0, 0.0, 0.0},
                .orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 0.9}};
            crossing_prediction.thickness = 5.0;
            crossing_prediction.scale_invariant = true;
            crossing_prediction.color =
                foxglove::schemas::Color{1.0, 0.0, 0.1, crossing_probability};

            foxglove::schemas::Point3 point;
            point.x = position.x;
            point.y = -lane_width / 2.0 - 1;
            point.z = 0;
            crossing_prediction.points.push_back(point);
            point.x = position.x;
            point.y = lane_width / 2.0 + 1;
            point.z = 0;
            crossing_prediction.points.push_back(point);
            entity.lines.push_back(crossing_prediction);

            // continue prediction line
            foxglove::schemas::LinePrimitive continue_prediction;
            continue_prediction.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
            continue_prediction.pose = foxglove::schemas::Pose{
                .position = foxglove::schemas::Vector3{0.0, 0.0, 0.0},
                .orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 0.9}};
            continue_prediction.thickness = 5.0;
            continue_prediction.scale_invariant = true;
            continue_prediction.color =
                foxglove::schemas::Color{0.0, 1.0, 0.1, 1 - crossing_probability};

            point.x = start_position.x - lane_width / 2.0 - 1.5;
            point.y = position.y;
            point.z = 0;
            continue_prediction.points.push_back(point);
            point.x = start_position.x + lane_width / 2.0 + 1.5;
            point.y = position.y;
            point.z = 0;
            continue_prediction.points.push_back(point);
            entity.lines.push_back(continue_prediction);

            // text
            foxglove::schemas::TextPrimitive text_maker;
            std::string text_str = std::to_string(i) + ":" + pedestrian->state_string() + "\n";
            {
                std::ostringstream oss;
                oss.precision(2);
                oss << std::fixed << crossing_probability;
                text_str += "Prob:" + oss.str();
            }
            text_maker.text = text_str;
            text_maker.font_size = 15.0;
            text_maker.billboard = true;
            text_maker.scale_invariant = true;
            text_maker.color = foxglove::schemas::Color{1.0, 1.0, 1.0, 1.0};
            text_maker.pose = foxglove::schemas::Pose{
                .position = foxglove::schemas::Vector3{position.x, position.y, 3.0},
                .orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 1.0}};
            entity.texts.push_back(text_maker);

            pedestrian_entities.push_back(entity);
        }
    }

    foxglove::schemas::SceneUpdate pedestrian_update;
    pedestrian_update.entities = pedestrian_entities;

    return pedestrian_update;
}
