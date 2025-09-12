/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-11 23:35:58
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-13 15:25:09
 * @FilePath: /dive-into-contingency-planning/planning/qp_tree_solver_osqp.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "planning/qp_tree_solver_osqp.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <spdlog/spdlog.h>

QPTreeSolverOSQP::QPTreeSolverOSQP(const MPCModel& mpc, double u_min, double u_max)
    : QP_tree_solver_base(mpc, u_min, u_max), control_bounds_in_A_(true) {}

Eigen::VectorXd QPTreeSolverOSQP::solve(const Eigen::Vector2d& x0, const Eigen::Vector2d& xd,
                                        const Constraints& k, int n_steps,
                                        const std::vector<std::vector<int>>& varss,
                                        const std::vector<std::vector<double>>& scaless) {
    // build matrices
    S = mpc.get_S(n_steps, varss);
    T = mpc.get_T(n_steps, varss);

    Q_bar = mpc.get_Q_bar(n_steps, varss, scaless);
    R_bar = mpc.get_R_bar(n_steps, varss, scaless);

    H = 2 * (R_bar + S.transpose() * Q_bar * S);
    F = 2 * (T.transpose() * Q_bar * S);
    G = 2 * Q_bar * S;

    const Eigen::VectorXd& Xd = mpc.get_Xd(xd, n_steps);
    C = (x0.transpose() * F).transpose() - (Xd.transpose() * G).transpose();

    // Constraints
    const Eigen::VectorXd Xmax = k.getXmax();
    const Eigen::MatrixXd Sextract = k.getSextract();

    if (!KA.rows()) {
        if (control_bounds_in_A_) {
            KA.resize(Sextract.rows() + H.rows(), H.rows());
            Up.resize(Sextract.rows() + H.rows());
            Lo.resize(Sextract.rows() + H.rows());
        } else {
            KA.resize(Sextract.rows(), H.rows());
            Up.resize(Sextract.rows());
            Lo.resize(Sextract.rows());
        }
    }

    // traj constraints
    KA.block(0, 0, Sextract.rows(), H.rows()) = Sextract * S;
    Up.head(Sextract.rows()) = Sextract * (Xmax - T * x0);
    Lo.head(Up.rows()) =
        Eigen::VectorXd::Constant(Up.rows(), std::numeric_limits<double>::lowest());

    // control bounds constraints
    if (control_bounds_in_A_) {
        KA.block(Sextract.rows(), 0, H.rows(), H.rows()) =
            Eigen::MatrixXd::Identity(H.rows(), H.rows());
        Up.tail(H.rows()) = Eigen::VectorXd::Constant(H.rows(), u_max_);
        Lo.tail(H.rows()) = Eigen::VectorXd::Constant(H.rows(), u_min_);
    }

    return call_osqp_solver();
}

Eigen::VectorXd QPTreeSolverOSQP::call_osqp_solver() {
    const int n = static_cast<int>(H.rows());
    const int m = static_cast<int>(KA.rows());

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    // tune more settings here if needed, e.g., warm start, tolerances, polish
    // solver.settings()->setWarmStart(true);
    Eigen::SparseMatrix<double> hessian = H.sparseView();
    Eigen::SparseMatrix<double> linearMatrix = KA.sparseView();
    solver.data()->setNumberOfVariables(n);
    solver.data()->setNumberOfConstraints(m);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(C);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(Lo);
    solver.data()->setUpperBound(Up);

    // Initialize and solve
    if (!solver.initSolver()) {
        spdlog::error("[planning] Failed to initialize OSQP solver.");
        return Eigen::VectorXd(n);
    }
    solver.solveProblem();

    // Retrieve solution (empty if not solved)
    if (solver.getStatus() != OsqpEigen::Status::Solved) {
        spdlog::error("[planning] OSQP solver failed with status: {}",
                      static_cast<int>(solver.getStatus()));
        return Eigen::VectorXd(n);
    }
    Eigen::VectorXd U_sol = solver.getSolution();
    if (U_sol.size() != n) {
        spdlog::error("[planning] OSQP solver returned solution of incorrect size: {}, expected {}",
                      U_sol.size(), n);
        return Eigen::VectorXd(n);
    }

    return U_sol;
}
