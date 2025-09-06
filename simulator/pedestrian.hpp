#pragma once

#include "common/common.hpp"
#include "foxglove/schemas.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

class Pedestrian {
  public:
    Pedestrian(uint32_t id, const State& start_position)
        : id_(id), start_position_(start_position), current_state_(start_position) {
        last_update_time_ = TimeUtil::NowSeconds();
    }

    virtual double crossing_probability(double now_s, double now_x) const = 0;
    virtual void step(double now_s, double now_x) = 0;
    virtual bool is_forward_direction() const = 0;

    State get_start_position() const { return start_position_; }

    State get_position() const { return current_state_; }

    uint32_t id() const { return id_; }

    bool is_done() const { return state_ == PedestrianState::DONE; }

    bool is_uncertain() const { return state_ == PedestrianState::UNCERTAIN; }

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
          crossing_duration_(4.0),
          forward_(forward) {}

    bool is_forward_direction() const { return forward_; }

    bool is_crossing(double time, double x);

    double crossing_probability(double time, double x) const;

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

    double crossing_probability(double time, double x) const;

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

    std::vector<std::shared_ptr<Pedestrian>> pedestrians() const { return pedestrians_; }

    std::shared_ptr<Pedestrian> pedestrian(std::size_t i) const { return pedestrians_[i]; }

    void erase(std::size_t i) { pedestrians_[i] = nullptr; }

    void replace_pedestrian(std::size_t i, const std::shared_ptr<Pedestrian>& pedestrian) {
        pedestrians_[i] = pedestrian;
    }

    void step(const State& car_position);

  private:
    std::vector<std::shared_ptr<Pedestrian>> pedestrians_;
};

std::shared_ptr<Pedestrian> generate_new_pedestrian(double p_crossing, uint32_t id,
                                                    double lane_width, const State& car_position);
