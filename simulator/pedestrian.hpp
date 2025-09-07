/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-06 19:25:09
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 17:44:35
 * @FilePath: /dive-into-contingency-planning/simulator/pedestrian.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "foxglove/schemas.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <vector>

class Pedestrian {
  public:
    Pedestrian(uint32_t id, const State& start_position)
        : id_(id), start_position_(start_position), current_state_(start_position) {
        last_update_time_ = TimeUtil::NowSeconds();
    }

    virtual double get_crossing_probability() const = 0;
    virtual void step(double now_s, double now_x) = 0;
    virtual bool is_forward_direction() const = 0;

    State get_start_position() const { return start_position_; }

    State get_position() const { return current_state_; }

    uint32_t id() const { return id_; }

    bool is_done() const { return state_ == PedestrianState::DONE; }

    bool is_uncertain() const { return state_ == PedestrianState::UNCERTAIN; }

    std::string state_string() const {
        switch (state_) {
            case PedestrianState::UNCERTAIN:
                return "UNCERTAIN";
            case PedestrianState::MOVING:
                return "MOVING";
            case PedestrianState::DONE:
                return "DONE";
            default:
                return "UNKNOWN";
        }
    }

  protected:
    enum class PedestrianState { UNCERTAIN = 0, MOVING, DONE } state_{PedestrianState::UNCERTAIN};

    uint32_t id_;
    State start_position_{0, 0, 0, 0};
    State current_state_{0, 0, 0, 0};
    double last_update_time_ = 0.0;
};

class CrossingPedestrian : public Pedestrian {
  public:
    CrossingPedestrian(uint32_t id, const State& start_position, double p, double crossing_x,
                       bool forward)
        : Pedestrian(id, start_position),
          p_(p),
          crossing_x_(crossing_x),
          crossing_duration_(5.0),
          forward_(forward) {}

    bool is_forward_direction() const { return forward_; }

    bool is_crossing(double time, double x);

    double get_crossing_probability() const;

    void step(double now_s, double now_x);

  private:
    double p_ = 0.5;
    double crossing_time_ = 0.0;
    double crossing_x_ = 0.0;
    double crossing_duration_ = 0.0;
    bool forward_ = true;
};

class NonCrossingPedestrian : public Pedestrian {
  public:
    NonCrossingPedestrian(uint32_t id, const State& start_position, double p, double non_crossing_x,
                          bool forward)
        : Pedestrian(id, start_position),
          p_(p),
          non_crossing_x_(non_crossing_x),
          non_crossing_duration_(4.0),
          forward_(forward) {}

    bool is_forward_direction() const { return forward_; }

    bool is_non_crossing(double time, double x);

    double get_crossing_probability() const;

    void step(double now_s, double now_x);

  private:
    double p_ = 0.5;
    double non_crossing_time_ = 0.0;
    double non_crossing_x_ = 0.0;
    double non_crossing_duration_ = 0.0;
    bool forward_ = true;
};

class PedestrianObserver {
  public:
    PedestrianObserver() = delete;
    PedestrianObserver(std::size_t N) : pedestrians_(N) {}

    foxglove::schemas::SceneUpdate observe_pedestrians(const State& car_position,
                                                       double lane_width) const;

    // Return a copy of internal vector under shared lock.
    std::vector<std::shared_ptr<Pedestrian>> pedestrians() const {
        std::shared_lock lock(mutex_);
        return pedestrians_;
    }

    std::shared_ptr<Pedestrian> pedestrian(std::size_t i) const {
        std::shared_lock lock(mutex_);
        return pedestrians_[i];
    }

    void erase(std::size_t i) {
        std::unique_lock lock(mutex_);
        pedestrians_[i] = nullptr;
    }

    void replace_pedestrian(std::size_t i, const std::shared_ptr<Pedestrian>& pedestrian) {
        std::unique_lock lock(mutex_);
        pedestrians_[i] = pedestrian;
    }

    void step(const State& car_position);

    // Return a thread-safe snapshot of pedestrians as const pointers so callers cannot
    // mutate them. The snapshot is created under a shared lock to synchronize with writers.
    std::vector<std::shared_ptr<const Pedestrian>> snapshot_pedestrians() const {
        std::shared_lock lock(mutex_);
        std::vector<std::shared_ptr<const Pedestrian>> out;
        out.reserve(pedestrians_.size());
        for (const auto& p : pedestrians_) {
            if (p) {
                out.push_back(std::static_pointer_cast<const Pedestrian>(p));
            }
        }
        return out;
    }

  private:
    mutable std::shared_mutex mutex_;
    std::vector<std::shared_ptr<Pedestrian>> pedestrians_;
};

std::shared_ptr<Pedestrian> generate_new_pedestrian(double p_crossing, uint32_t id,
                                                    double lane_width, const State& car_position);
