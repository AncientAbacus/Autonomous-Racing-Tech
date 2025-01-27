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
 * @file lqr_controller.cpp
 * @author Dominic Nightingale (djnighti@ucsd.edu)
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @brief
 * @version 0.2
 * @date 2022-1-4
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>
#include "Eigen/Core"

#include "lqg_controller/lqg_controller.hpp"

namespace race
{
LqgController::LqgController(
  LqgControllerConfig::SharedPtr lqg_config,
  LqrCalculatorConfig::SharedPtr lqr_config,
  LkfCalculatorConfig::SharedPtr lkf_config)
: config_(lqg_config),
  art_lqr(lqr_config),
  art_lkf(lkf_config)
{
  for (int i = 0; i < 10; i++) {
    filter_vx.push_back(0.);
    filter_vtheta.push_back(0.);
  }

  for (int i = 0; i < 1; i++) {
    filter_x.push_back(0.);
    filter_theta.push_back(0.);
  }

  state_ = std::make_shared<LqgState>();
  d_max = config_->max_steer_rad;
  d_min = -1.0 * d_max;
  X = Eigen::MatrixXd::Zero(3, 1);
  PATH = Eigen::MatrixXd();
  x = X(0, 0);
  y = X(1, 0);
  yaw = X(2, 0);
  P0 = Eigen::MatrixXd(4, 4);
  x0 = Eigen::MatrixXd(4, 1);
  x0(0, 0) = config_->initial_state.x1;
  x0(1, 0) = config_->initial_state.x2;
  x0(2, 0) = config_->initial_state.x3;
  x0(3, 0) = config_->initial_state.x4;
  P0(0, 0) = config_->initial_error_cov.p0_1;
  P0(1, 1) = config_->initial_error_cov.p0_2;
  P0(2, 2) = config_->initial_error_cov.p0_3;
  P0(3, 3) = config_->initial_error_cov.p0_4;
  Ts = config_->ts;
  state_est = x0;
  P = P0;
  state_measurement = x0;
}

void LqgController::recalculate_params()
{
  art_lqr.recalculate_params();
}

double LqgController::lqg_step(double adj_ratio)
{
  // TODO(haoru): make sure there is check in the rvc
  if (!this->PATH.size() || !this->X.size()) {
    return 0.0;
  }

  // Update control gains
  auto K = art_lqr.lqr_step(vx, A, B);

  // Calculate current measurement model
  // TODO(dominic): rewrite in CPP this->art_car_model.calc_output(this->state_measurement)

  // Steering angle output
  // state_est = state_measurement;
  if (config_->use_pure_pursuit_params) {
    auto dist = std::max(
      art_lqr.get_config()->state_performance.l_min,
      art_lqr.get_config()->state_performance.l_factor * vx);
    K(0, 0) = 2 * config_->LEN / (dist * dist);
    K(0, 2) = 2 * config_->LEN / (dist);
  }
  if (!config_->use_differential_state_params) {
    K(0, 1) = 0.;
    K(0, 3) = 0.;
  }
  K(0, 0) *= config_->scale_factor_x;
  K(0, 2) *= config_->scale_factor_theta;
  double delta_nc = -(K * state_est)(0, 0) + this->D(3, 0) +
    config_->steady_state_error_correction_multiplier * K(0, 2) * this->D(1, 0);
  // std::cout << K << std::endl;
  state_->lqg_delta = delta_nc;
  double delta = delta_constraints(delta_nc);
  Eigen::MatrixXd u(1, 1);
  u(0, 0) = -(K * state_est)(0, 0);

  Eigen::MatrixXd y_measure = C * state_measurement;
  art_lkf.lkf_step(A, B, C, state_est, u, y_measure, P);
  state_->lqg_e_cg = state_measurement(0, 0);
  state_->lqg_e_cg_dot = state_measurement(1, 0);
  state_->lqg_theta_e = state_measurement(2, 0);
  state_->lqg_theta_e_dot = state_measurement(3, 0);

  state_->lqg_e_cg_hat = state_est(0, 0);
  state_->lqg_e_cg_dot_hat = state_est(1, 0);
  state_->lqg_theta_e_hat = state_est(2, 0);
  state_->lqg_theta_e_dot_hat = state_est(3, 0);

  state_->yaw = this->D(3, 0);
  state_->vx = vx;

  // Get new state measurements
  // update_states();

  return delta;
}

void LqgController::set_speed(const double & speed_mps)
{
  vx = std::clamp(speed_mps, config_->min_speed_mps, config_->max_speed_mps);
}

void LqgController::set_transformation(
  const double & x, const double & y,
  const double & heading_rad)
{
  this->x = x;
  this->y = -y;
  this->yaw = heading_rad - angle_offset;
}

void LqgController::set_cross_track_error(const double & cross_track_error, double curr_time)
{
  if (abs(state_measurement(0, 0) + cross_track_error) > 1e-9) {
    if (abs(curr_time - last_time_x) > 1e-5) {
      this->insert_val(
        filter_vx,
        ((-cross_track_error - state_measurement(0, 0)) / (curr_time - last_time_x)));
    }
    this->insert_val(filter_x, -cross_track_error);
    state_measurement(0, 0) = std::reduce(filter_x.begin(), filter_x.end()) / filter_x.size();
    state_measurement(1, 0) = std::reduce(filter_vx.begin(), filter_vx.end()) / filter_vx.size();

    last_time_x = curr_time;
  }
}

// TODO(haoru): revise implementation
void LqgController::insert_val(std::vector<double> & v, double val)
{
  for (int i = 0; i < v.size() - 1; i++) {
    v[i] = v[i + 1];
  }
  v[v.size() - 1] = val;
}

void LqgController::set_path(PathSharedPtr path, double curr_time)
{
  PATH = Eigen::MatrixXd(path->size(), 2);
  for (int i = 0; i < path->size(); i++) {
    PATH(i, 0) = -(*path)[i].location.y;
    PATH(i, 1) = (*path)[i].location.x;
  }
  xp = PATH.col(0);
  yp = PATH.col(1);
  if (abs(-path->front().target_yaw - state_measurement(2, 0)) > 1e-9) {
    if (abs(curr_time - last_time_theta) > 1e-5) {
      this->insert_val(
        filter_vtheta,
        ((-path->front().target_yaw - state_measurement(2, 0)) / (curr_time - last_time_theta)));
    }
    this->insert_val(filter_theta, -path->front().target_yaw);
    state_measurement(2, 0) =
      std::reduce(filter_theta.begin(), filter_theta.end()) / filter_theta.size();
    state_measurement(3, 0) =
      std::reduce(filter_vtheta.begin(), filter_vtheta.end()) / filter_vtheta.size();
    last_time_theta = curr_time;
  }
}

void LqgController::set_model(
  Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
  Eigen::MatrixXd D)
{
  this->A = A;
  this->B = B;
  this->C = C;
  this->D = D;
}

void LqgController::update_states()
{
  state_measurement(1, 0) = 0.0;
  state_measurement(3, 0) = 0.0;
}

double LqgController::delta_constraints(double steering_angle)
{
  if (steering_angle > d_max) {
    steering_angle = d_max;
  } else if (steering_angle < d_min) {
    steering_angle = d_min;
  }
  return steering_angle;
}

void LqgController::get_state(LqgState::SharedPtr & lqg_state)
{
  lqg_state = state_;
}

}  // namespace race
