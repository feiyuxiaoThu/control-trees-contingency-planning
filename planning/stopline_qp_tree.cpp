/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 16:00:13
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-14 23:54:46
 * @FilePath: /dive-into-contingency-planning/planning/stopline_qp_tree.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "planning/stopline_qp_tree.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>

StopLineQPTree::StopLineQPTree(int n_branches, int steps_per_phase)
    : n_branches_(n_branches),
      steps_(steps_per_phase),
      model_(1.0 / steps_per_phase, 1.0, 5.0),
      v_desired_(50 / 3.6),
      stoplines_(n_branches_ > 1 ? n_branches_ - 1 : 1),
      optimization_run_(false),
      optimization_error_(false) {
    init_logger();
    solver_ = std::make_unique<QPTreeSolverOSQP>(model_, u_min_, u_max_);
    LOG_INFO(logger_, "Using OSQP solver");
}

StopLineQPTree::StopLineQPTree(const YAML::Node& config)
    : n_branches_(config["n_branches"].as<int>(5)),
      steps_(config["steps_per_phase"].as<int>(4)),
      u_min_(config["u_min"].as<double>(-6.0)),
      u_max_(config["u_max"].as<double>(2.0)),
      v_desired_(config["v_desired"].as<double>(50 / 3.6)),
      model_(1.0 / steps_, 1.0, 5.0),
      stoplines_(n_branches_ > 1 ? n_branches_ - 1 : 1),
      optimization_run_(false),
      optimization_error_(false) {
    init_logger(config["log_level"].as<std::string>("info"));
    if (config["solver_type"].as<std::string>("osqp") == "osqp") {
        solver_ = std::make_unique<QPTreeSolverOSQP>(model_, u_min_, u_max_);
        LOG_INFO(logger_, "Using OSQP solver");
    } else {
        solver_ = std::make_unique<QPTreeSolverDec>(model_, u_min_, u_max_);
        LOG_INFO(logger_, "Using Control-Tree decentralized solver");
    }
}

void StopLineQPTree::update_stopline(const State& ego_current_state,
                                     std::vector<std::shared_ptr<const Pedestrian>> pedestrians) {
    const size_t N = pedestrians.size();
    stoplines_.resize(N);

    for (size_t i = 0; i < N; ++i) {
        const double stop = pedestrians[i]->get_position().x - 6.5;
        const double probability = pedestrians[i]->get_crossing_probability();

        if (ego_current_state.x < stop + 1 && probability > 0.01) {
            stoplines_[i].x = stop;
            stoplines_[i].p = probability;
        } else {
            stoplines_[i].x = std::numeric_limits<double>::infinity();
            stoplines_[i].p = 0;
        }
    }

    // sort stoplines
    std::sort(stoplines_.begin(), stoplines_.end(),
              [](const Stopline& a, const Stopline& b) { return a.x < b.x; });

    for (auto i = 0; i < stoplines_.size(); ++i) {
        LOG_INFO(logger_, "{}--th stopline {:.2f} prob {:.2f}", i, stoplines_[i].x,
                 stoplines_[i].p);
    }

    create_tree();
}

TimeCostPair StopLineQPTree::plan(const State& current_state,
                                  std::vector<std::shared_ptr<const Pedestrian>> pedestrians) {
    if (!solver_) {
        LOG_ERROR(logger_, "Solver not initialized!");
        return {0.0, 0.0};
    }

    ++plan_seq_;
    update_stopline(current_state, pedestrians);

    // INITIAL STATES
    x0_ = Eigen::Vector2d();
    x0_ << 0, current_state.velocity;

    // DESIRED
    Eigen::Vector2d xd;
    xd << 0, v_desired_;

    // CONSTRAINT
    Constraints k(tree_->n_steps, tree_->varss);
    for (auto i = 0; i < (n_branches_ > 1 ? n_branches_ - 1 : 1); ++i) {
        double xmax = 0;
        // far behind
        if (stoplines_[i].p < 0.01 || current_state.x > stoplines_[i].x + 1) {
            xmax = 1000, 0;  // rather remove constraint!
        } else {
            xmax = stoplines_[i].x - current_state.x;
        }

        k.add_constraint(i, Eigen::Vector2d(xmax, 0), Eigen::Vector2d(1, 0));
    }

    auto start = std::chrono::high_resolution_clock::now();

    auto U = solver_->solve(x0_, xd, k, tree_->n_steps, tree_->varss, tree_->scaless);

    auto end = std::chrono::high_resolution_clock::now();
    double execution_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    solve_cost_time_ms_ = execution_time_us / 1000.0;
    LOG_INFO(logger_, "execution time (ms):{}", solve_cost_time_ms_);

    if (std::fabs(U[1] - U[0]) > 0.5) {
        LOG_WARN(logger_, "High Jerk at trajectory start!!!");
        LOG_INFO(logger_, "x: {} v: {}", current_state.x, current_state.velocity);
    }

    optimization_error_ = !validate_and_save_solution(U, current_state);

    if (optimization_error_) {
        LOG_INFO(logger_, "Generate control for emergency brake, o.x: {} o.v: {} v_desired_: {}",
                 current_state.x, current_state.velocity, v_desired_);

        const auto& U = emergency_brake(current_state.velocity, *tree_, steps_, u_min_);

        bool ok_emergency_brake = validate_and_save_solution(U, current_state);

        if (!ok_emergency_brake) {
            LOG_ERROR(logger_, "Planning error :((");
        }
    }

    solution_cost_ = model_.cost(x0_, U_sol_, xd);

    LOG_INFO(logger_, "costs: {:.2f} speed: {:.2f} accel: {:.2f}", solution_cost_,
             current_state.velocity, U_sol_[0]);

    return {solve_cost_time_ms_, solution_cost_};
}

bool StopLineQPTree::validate_and_save_solution(const Eigen::VectorXd& U, const State& state) {
    if (U.rows()) {
        // logger_->info("Solved :)");
        auto x0 = x0_;
        x0(0) += state.x;

        auto X = model_.predict_trajectory(x0, U, tree_->varss);

        if (valid(U, X)) {
            U_sol_ = U;
            X_sol_ = X;
            optimization_run_ = true;

            return true;
        } else {
            LOG_WARN(logger_, "Optimization succeeded but invalid trajectory");
        }
    } else {
        LOG_ERROR(logger_, "Solution infeasible :(");
    }

    return false;
}

planning::protos::PlanningInfo StopLineQPTree::get_debug_result(const State& current_state) {
    planning::protos::PlanningInfo info;
    auto mutable_header = info.mutable_header();
    auto timestamp = TimeUtil::NowTimestamp();
    mutable_header->set_seq(plan_seq_);
    mutable_header->set_frame_id("map");
    mutable_header->mutable_stamp()->set_sec(timestamp.sec);
    mutable_header->mutable_stamp()->set_nsec(timestamp.nsec);
    auto mutable_state = info.mutable_current_state();
    mutable_state->set_pos_x(current_state.x);
    mutable_state->set_pos_y(current_state.y);
    mutable_state->set_theta(current_state.yaw);
    mutable_state->set_velocity(current_state.velocity);
    info.set_cruise_velocity(to_fixed<2>(v_desired_));

    if (!optimization_run_ || optimization_error_) {
        return info;
    }

    double target_accel = U_sol_.size() > 0 ? U_sol_[0] : 0.0;
    info.set_target_accel(to_fixed<2>(target_accel));
    info.set_cost_time_ms(to_fixed<2>(solve_cost_time_ms_));
    info.set_solution_cost(to_fixed<2>(solution_cost_));

    for (auto l = 0; l < tree_->varss.size(); ++l) {
        auto speed_profile = info.add_solution();
        speed_profile->set_branch_id(l);
        speed_profile->set_probability(to_fixed<2>(tree_->scaless[l].back()));
        double stopline_x = std::numeric_limits<double>::infinity();
        if (n_branches_ > 1 && l < stoplines_.size()) {
            // relative to current position
            stopline_x = to_fixed<2>(stoplines_[l].x - current_state.x);
        }
        speed_profile->set_stopline(stopline_x);
        speed_profile->mutable_points()->Reserve(tree_->varss[l].size());
        double current_odom = 0.0;
        double last_x = 0.0;
        size_t idx = 0;
        for (auto k : tree_->varss[l]) {
            auto speed_point = speed_profile->add_points();
            speed_point->set_x(X_sol_[2 * k]);
            speed_point->set_y(0.0);
            if (k == 0) {
                speed_point->set_s(0.0);
            } else {
                current_odom += (X_sol_[2 * k] - last_x);
                speed_point->set_s(current_odom);
            }
            speed_point->set_v(X_sol_[2 * k + 1]);
            speed_point->set_a(k < U_sol_.size() ? U_sol_[k] : 0.0);
            speed_point->set_t(1.0 * idx / steps_);
            last_x = X_sol_[2 * k];
            ++idx;
        }
    }

    return info;
}

void StopLineQPTree::create_tree() {
    if (n_branches_ == 1) {
        tree_ = TreePb::refined(std::make_shared<Tree1Branch>(), steps_);
    } else {
        const auto ps = fuse_probabilities(stoplines_, n_branches_ - 1);

        assert(ps.size() == n_branches_ - 1 && "discrepancy in number of branches in control tree");

        tree_ = TreePb::refined(std::make_shared<TreeNBranches>(ps), steps_);
    }

    // debug info
    size_t branch_count = 0;
    LOG_DEBUG(logger_, "tree n_steps: {}", tree_->n_steps);
    LOG_DEBUG(logger_, "tree varss:");
    for (const auto& vars : tree_->varss) {
        LOG_DEBUG(logger_, "branch-{}: {}", branch_count++, fmt::join(vars, ", "));
    }
    branch_count = 0;
    LOG_DEBUG(logger_, "tree scaless:");
    for (const auto& scales : tree_->scaless) {
        LOG_DEBUG(logger_, "branch-{}: {:.2f}", branch_count++, fmt::join(scales, ", "));
    }

    assert(tree_->scaless.size() == n_branches_ &&
           "discrepancy in number of branches in control tree");
}

bool StopLineQPTree::valid(const Eigen::VectorXd& U, const Eigen::VectorXd& X) const {
    const double eps = 0.15;

    for (auto i = 0; i < U.rows(); ++i) {
        if (std::isnan(U[i])) {
            LOG_ERROR(logger_, "nan in U vector!");
            return false;
        }

        if (!(u_min_ - eps <= U[i] && U[i] <= u_max_ + eps)) {
            LOG_ERROR(logger_, "control out of bounds!: {}", U[i]);
            return false;
        }
    }

    for (auto i = 0; i < X.rows(); ++i) {
        if (std::isnan(X[i])) {
            LOG_ERROR(logger_, "nan in X vector!");
            return false;
        }
    }

    return true;
}

void StopLineQPTree::init_logger(const std::string& log_level_str /* = "info" */) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    logger_ = std::make_shared<spdlog::logger>("stop_line_logger", console_sink);
    logger_->set_level(spdlog::level::from_str(log_level_str)); 
    logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^\033[1m%l\033[0m%$] [%s:%#] %v");
}

Eigen::VectorXd emergency_brake(const double v, const TreePb& tree, int steps_per_phase, double u) {
    Eigen::VectorXd U = Eigen::VectorXd::Zero(tree.n_steps);

    for (auto vars : tree.varss) {
        double v_loop = v;

        for (auto i = 0; i < vars.size(); ++i) {
            double remaining_braking_time = fabs(v_loop / u);

            if (remaining_braking_time > 1.0 / steps_per_phase) {
                U[vars[i]] = u;
                v_loop += u / steps_per_phase;
            } else {
                // last step
                U[vars[i]] = -v_loop / steps_per_phase;
                v_loop = 0;
                break;
            }
        }
    }

    return U;
}

std::vector<double> fuse_probabilities(const std::vector<Stopline>& stoplines, int n) {
    std::vector<double> probabilities(n);

    double q = 1.0;
    for (auto i = 0; i < n; ++i) {
        probabilities[i] = q * stoplines[i].p;
        q *= (1 - stoplines[i].p);
    }

    for (auto j = n; j < stoplines.size(); ++j) {
        probabilities[n - 1] += q * stoplines[j].p;
        q *= (1 - stoplines[j].p);
    }

    return probabilities;
}

Eigen::VectorXd emergency_brake(double v, int n_phases, int steps_per_phase, double u) {
    Eigen::VectorXd U = Eigen::VectorXd::Zero(n_phases * steps_per_phase);

    for (auto i = 0; i < U.rows(); ++i) {
        double remaining_braking_time = fabs(v / u);

        if (remaining_braking_time > 1.0 / steps_per_phase) {
            U[i] = u;
            v += u / steps_per_phase;
        } else {
            // last step
            U[i] = -v / steps_per_phase;
            v = 0;
            break;
        }
    }

    return U;
}
