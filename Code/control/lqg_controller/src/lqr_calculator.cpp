// Copyright 2022 AI Racing Tech
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


/**
 * @file lqr_calculator.cpp
 * @author Dominic Nightingale (djnighti@ucsd.edu)
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @brief
 * @version 0.2
 * @date 2022-7-15
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <iostream>
#include <algorithm>

#include "Eigen/LU"
#include "Eigen/SVD"
#include "lqg_controller/lqr_calculator.hpp"
namespace race
{

void LqrCalculator::recalculate_params()
{
  const auto & sp = config_->state_performance;
  this->l_factor = sp.l_factor;
  this->l_const = sp.l_const;
  this->rc_1_val = sp.rc_1_val;
  this->Ts = config_->Ts;
  this->Vmin = config_->Vmin;
  this->ld_min = sp.l_min;
  this->Vmax = config_->Vmax;
  this->are_max_iter = config_->are_max_iter;
  this->are_error_thresh = config_->are_error_thresh;
  this->q_range = Eigen::MatrixXd(3, 1);
  this->V_range = Eigen::MatrixXd(3, 1);
  this->qc_1_data = this->qc_1_coeff * this->q_range;
  this->qc_2_data = this->qc_1_coeff * this->Ts * this->q_range;
  this->qc_3_data = this->qc_3_coeff * this->q_range;
  this->qc_4_data = this->qc_3_coeff * this->Ts * this->q_range;
  // these values are computed once upon initialization
  this->Qc = Eigen::MatrixXd(4, 4);
  this->Rc = Eigen::MatrixXd(1, 1);
  // this->compute_all_powerfit_coeff();
}

LqrCalculator::LqrCalculator(LqrCalculatorConfig::SharedPtr config)
: config_(config)
{
  const auto & sp = config_->state_performance;
  this->rc_1_val = sp.rc_1_val;
  this->Ts = config->Ts;
  this->Vmin = config->Vmin;
  this->Vmax = config->Vmax;
  this->are_max_iter = config->are_max_iter;
  this->are_error_thresh = config->are_error_thresh;
  this->q_range = Eigen::MatrixXd(3, 1);
  this->V_range = Eigen::MatrixXd(3, 1);
  this->l_factor = sp.l_factor;
  this->l_const = sp.l_const;
  this->Qc = Eigen::MatrixXd(4, 4);
  this->Rc = Eigen::MatrixXd(1, 1);
}

Eigen::MatrixXd LqrCalculator::lqr_step(double vx, Eigen::MatrixXd A, Eigen::MatrixXd B)
{
  compute_weights(vx);
  Eigen::MatrixXd K;
  Eigen::MatrixXd S = Qc;
  Eigen::MatrixXd S_prev;
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  double S_diff = 1.0;
  // cout << this->Qc << endl;
  for (int n = 0; n < config_->are_max_iter; n++) {
    S_prev = S;
    S = AT * S * A - (AT * S * B) * (BT * S * B + this->Rc).inverse() * BT * S * A + this->Qc;
    S_diff = abs(
      S.jacobiSvd().singularValues().maxCoeff() - S_prev.jacobiSvd().singularValues().maxCoeff()
    );
    K = (this->Rc + BT * S * B).inverse() * BT * S;
    if (S_diff < config_->are_error_thresh) {
      break;
    }
  }

  return K;
}

LqrCalculatorConfig::SharedPtr LqrCalculator::get_config()
{
  return this->config_;
}

Eigen::MatrixXd LqrCalculator::compute_a_powerfit_coeff(Eigen::MatrixXd x, Eigen::MatrixXd y)
{
  // this function only needs to be called upon initialization for each state weight
  // x: speed vector
  // y: state weight vector
  Eigen::MatrixXd X = (x.array().log()).matrix();
  Eigen::MatrixXd Y = (y.array().log()).matrix();
  Eigen::MatrixXd A = Eigen::MatrixXd(2, 2);
  Eigen::MatrixXd b = Eigen::MatrixXd(2, 1);
  Eigen::MatrixXd phi = Eigen::MatrixXd(2, 1);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      A(i, j) = X.array().pow(j + i).sum();
    }
    b(i) = (X.array().pow(i) * Y.array()).sum();
  }
  phi = A.inverse() * b;
  phi(0) = pow(10, phi(0));
  return phi;
}


double LqrCalculator::power_function_eval(double vx, double a, double b)
{
  return a * pow(vx, b);
}

void LqrCalculator::compute_weights(double vx)
{
  Eigen::Matrix<double, 1, 1> Rc_diag;
  Rc_diag << this->rc_1_val;

  double ld = std::max(this->ld_min, this->l_factor * vx + this->l_const);
  this->Qc(0, 0) = 1;
  this->Qc(0, 1) = 0;
  this->Qc(0, 2) = ld;
  this->Qc(0, 3) = 0;

  this->Qc(1, 0) = 0;
  this->Qc(1, 1) = 1;
  this->Qc(1, 2) = 0;
  this->Qc(1, 3) = 0;

  this->Qc(2, 0) = ld;
  this->Qc(2, 1) = 0;
  this->Qc(2, 2) = ld * ld;
  this->Qc(2, 3) = 0;

  this->Qc(3, 0) = 0;
  this->Qc(3, 1) = 0;
  this->Qc(3, 2) = 0;
  this->Qc(3, 3) = 1;
  this->Qc = this->Qc / (1 + 2 * ld + ld * ld);
  this->Rc = Rc_diag.asDiagonal() * ld * ld;
}
}  // namespace race
