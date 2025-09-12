/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 16:00:13
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-13 15:49:10
 * @FilePath: /dive-into-contingency-planning/planning/stopline_qp_tree.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "planning/stopline_qp_tree.hpp"

#include <spdlog/spdlog.h>

StopLineQPTree::StopLineQPTree(int n_branches, int steps_per_phase)
    : n_branches_(n_branches),
      steps_(steps_per_phase),
      model_(1.0 / steps_per_phase, 1.0, 5.0),
      solver_(model_, u_min_, u_max_),
      v_desired_(50 / 3.6),
      stoplines_(n_branches_ > 1 ? n_branches_ - 1 : 1),
      optimization_run_(false),
      optimization_error_(false) {
    create_tree();
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
        spdlog::info("{}--th stopline {:.2f} prob {:.2f}", i, stoplines_[i].x, stoplines_[i].p);
    }

    create_tree();
}

TimeCostPair StopLineQPTree::plan(const State& current_state,
                                  std::vector<std::shared_ptr<const Pedestrian>> pedestrians) {
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

    auto U = solver_.solve(x0_, xd, k, tree_->n_steps, tree_->varss, tree_->scaless);

    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    spdlog::info("[planning] execution time (ms):{}", execution_time_us / 1000);

    if (std::fabs(U[1] - U[0]) > 0.5) {
        spdlog::warn("[planning] High Jerk at trajectory start!!!");
        spdlog::info("[planning] x: {} v: {}", current_state.x, current_state.velocity);
    }

    optimization_error_ = !validate_and_save_solution(U, current_state);

    if (optimization_error_) {
        spdlog::info(
            "[planning] Generate control for emergency brake, o.x: {} o.v: {} v_desired_: {}",
            current_state.x, current_state.velocity, v_desired_);

        const auto& U = emergency_brake(current_state.velocity, *tree_, steps_, u_min_);

        bool ok_emergency_brake = validate_and_save_solution(U, current_state);

        if (!ok_emergency_brake) {
            spdlog::error("Planning error :((");
        }
    }

    double solution_cost = model_.cost(x0_, U_sol_, xd);
    spdlog::info("[planning] costs: {:.2f} speed: {:.2f} accel: {:.2f}", solution_cost,
                 current_state.velocity, U_sol_[0]);

    return {execution_time_us / 1000000, solution_cost};
}

bool StopLineQPTree::validate_and_save_solution(const Eigen::VectorXd& U, const State& state) {
    if (U.rows()) {
        // spdlog::info("Solved :)");
        auto x0 = x0_;
        x0(0) += state.x;

        auto X = model_.predict_trajectory(x0, U, tree_->varss);

        if (valid(U, X)) {
            U_sol_ = U;
            X_sol_ = X;

            optimization_run_ = true;

            return true;
        } else {
            spdlog::warn("[planning] Optimization succeeded but invalid trajectory");
        }
    } else {
        spdlog::error("[planning] Solution infeasible :(");
    }

    return false;
}

// std::vector<nav_msgs::Path> StopLineQPTree::get_trajectories()
// {
//     std::vector<nav_msgs::Path> paths(tree_->varss.size());

//     if(!optimization_run_ || optimization_error_)
//     {
//         return paths;
//     }

//     auto msg_from_s = [](double x)
//     {
//         tf2::Quaternion q;
//         q.setRPY(0, 0, 0);
//         geometry_msgs::PoseStamped pose;
//         pose.pose.position.x = x;
//         pose.pose.orientation.x = q.x();
//         pose.pose.orientation.y = q.y();
//         pose.pose.orientation.z = q.z();
//         pose.pose.orientation.w = q.w();
//         return pose;
//     };

//     for(auto l = 0; l < tree_->varss.size(); ++l)
//     {
//         {
//             nav_msgs::Path msg;
//             msg.header.stamp = ros::Time::now();
//             msg.header.frame_id = "map";
//             msg.poses.reserve(tree_->varss[l].size() + 1);

//             msg.poses.push_back(msg_from_s(x0_[0]));

//             if(l > 0 && tree_->scaless[l].back() < 0.01) // particular cas to avoid sending
//             degenerated trajs
//             {
//                 paths[l] = paths[l-1];
//             }
//             else
//             {
//                 // nominal case
//                 for(auto k : tree_->varss[l])
//                 {
//                     msg.poses.push_back(msg_from_s(X_sol_[2*k]));
//                 }

//                 paths[l] = msg;
//             }
//         }
//     }

//     return paths;
// }

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
    spdlog::debug("[planning] tree n_steps: {}", tree_->n_steps);
    spdlog::debug("[planning] tree varss:");
    for (const auto& vars : tree_->varss) {
        spdlog::debug("[planning] branch-{}: {}", branch_count++, fmt::join(vars, ", "));
    }
    branch_count = 0;
    spdlog::debug("[planning] tree scaless:");
    for (const auto& scales : tree_->scaless) {
        spdlog::debug("[planning] branch-{}: {:.2f}", branch_count++, fmt::join(scales, ", "));
    }

    assert(tree_->scaless.size() == n_branches_ &&
           "discrepancy in number of branches in control tree");
}

bool StopLineQPTree::valid(const Eigen::VectorXd& U, const Eigen::VectorXd& X) const {
    const double eps = 0.15;

    for (auto i = 0; i < U.rows(); ++i) {
        if (std::isnan(U[i])) {
            spdlog::error("[planning] nan in U vector!");
            return false;
        }

        if (!(u_min_ - eps <= U[i] && U[i] <= u_max_ + eps)) {
            spdlog::error("[planning] control out of bounds!: {}", U[i]);
            return false;
        }
    }

    for (auto i = 0; i < X.rows(); ++i) {
        if (std::isnan(X[i])) {
            spdlog::error("[planning] nan in X vector!");
            return false;
        }
    }

    return true;
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
