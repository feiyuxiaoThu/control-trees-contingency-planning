/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-09-07 15:56:05
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-09-07 15:56:19
 * @FilePath: /dive-into-contingency-planning/planning/qp_tree_problem_osqp.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved. 
 */

#pragma once

#include "planning/mpc_model.hpp"
#include "planning/qp_constraints.hpp"
#include "planning/qp_tree_solver_base.hpp"

#include <osqp/osqp.h>

csc * create_csc_matrix(const Eigen::MatrixXd & M);

class QP_tree_problem_OSQP : public QP_tree_joint_solver_base
{
public:
    QP_tree_problem_OSQP(const MPCModel & mpc,
                    double u_min, double u_max);

    ~QP_tree_problem_OSQP();

private:
    Eigen::VectorXd call_solver() override;

private:
    OSQPWorkspace *work;
    OSQPSettings  *settings;
    OSQPData      *data;
};
