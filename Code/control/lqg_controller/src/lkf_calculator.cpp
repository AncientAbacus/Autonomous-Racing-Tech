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
 * @file lkf_calculator.cpp
 * @author Dominic Nightingale (djnighti@ucsd.edu)
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @brief
 * @version 0.2
 * @date 2022-7-15
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#include <iostream>

#include "Eigen/LU"
#include "lqg_controller/lkf_calculator.hpp"

namespace race
{
LkfCalculator::LkfCalculator(LkfCalculatorConfig::SharedPtr config)
: config_(config)
{
  compute_weights();
}
void LkfCalculator::lkf_step(
  Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
  Eigen::MatrixXd & xhat, Eigen::MatrixXd u, Eigen::MatrixXd y,
  Eigen::MatrixXd & P)
{
  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
  Eigen::MatrixXd S_prev;
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd CT = C.transpose();

  // Kalman gain
  K = P * CT * (C * P * CT + Rf).inverse();

  // Prior state estimate
  xhat = A * xhat + B * u;

  // Posterior state estimate
  xhat = xhat + K * (y - C * xhat);

  // Prior Covariance
  P = P + this->Qf;

  // Posterior covariance
  P = (C - K * C) * P;
}

void LkfCalculator::compute_weights()
{
  Eigen::Matrix<double, 4, 1> Qf_diag;
  Qf_diag << config_->model_error.qf_1, config_->model_error.qf_2, config_->model_error.qf_3,
    config_->model_error.qf_4;
  Eigen::Matrix<double, 4, 1> Rf_diag;
  Rf_diag << config_->measurement_error.rf_1, config_->measurement_error.rf_2,
    config_->measurement_error.rf_3, config_->measurement_error.rf_4;
  Qf = Qf_diag.asDiagonal();
  Rf = Rf_diag.asDiagonal();
}
}  // namespace race
