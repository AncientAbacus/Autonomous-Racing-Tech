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
 * @file lqr_controller.hpp
 * @author Dominic Nightingale (djnighti@ucsd.edu)
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @brief
 * @version 0.2
 * @date 2022-1-4
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#ifndef LQG_CONTROLLER__LQG_CONTROLLER_HPP_
#define LQG_CONTROLLER__LQG_CONTROLLER_HPP_

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include "Eigen/Core"

#include "ttl.hpp"
#include "lqg_controller/lkf_calculator.hpp"
#include "lqg_controller/lqr_calculator.hpp"
#include "base_common/low_pass_filter.hpp"

namespace race
{
using race::ttl::Waypoint;
using race::ttl::Path;
using race::ttl::PathSharedPtr;
struct InitialState
{
  double x1;
  double x2;
  double x3;
  double x4;
};

struct InitialErrorCov
{
  double p0_1;
  double p0_2;
  double p0_3;
  double p0_4;
};
struct LqgControllerConfig
{
  typedef std::shared_ptr<LqgControllerConfig> SharedPtr;
  double ts;
  double max_steer_rad;
  double min_speed_mps;
  double max_speed_mps;
  double line_error_threshold;
  double scale_factor_x = 1.;
  double scale_factor_theta = 1.;
  double steady_state_error_correction_multiplier = 0.;
  double LEN = 2.97;
  bool use_pure_pursuit_params = false;
  bool use_differential_state_params = false;

  InitialState initial_state;
  InitialErrorCov initial_error_cov;
};

struct LqgState
{
  typedef std::shared_ptr<LqgState> SharedPtr;
  float lqg_delta;
  float d_ff;
  float lqg_speed;
  float lqg_e_cg;
  float lqg_e_cg_dot;
  float lqg_theta_e;
  float lqg_theta_e_dot;
  float future_curvature;
  float lqg_e_cg_hat;
  float lqg_e_cg_dot_hat;
  float lqg_theta_e_hat;
  float lqg_theta_e_dot_hat;
  float yaw;
  float yaw_rate;
  float vx;
  float vy;
};

class LqgController
{
public:
  typedef std::shared_ptr<LqgController> SharedPtr;
  explicit LqgController(
    LqgControllerConfig::SharedPtr lqg_config,
    LqrCalculatorConfig::SharedPtr lqr_config,
    LkfCalculatorConfig::SharedPtr lkf_config);
  double lqg_step(double adj_ratio);
  void set_speed(const double & speed_mps);
  void set_transformation(const double & x, const double & y, const double & heading_rad);
  void insert_val(std::vector<double> & v, double val);
  double get_val(std::vector<double> v);
  void set_cross_track_error(const double & cross_track_error, double curr_time);
  void set_path(PathSharedPtr path, double curr_time);
  void set_model(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D);
  void get_state(LqgState::SharedPtr & lqg_state);
  void recalculate_params();
  LowPassFilter output_filter_;

private:
  double d_max;
  double d_min;
  double v_max;
  double v_min;
  double V;
  double Ts;
  double angle_offset = M_PI / 2;
  double theta_path = 0.0;
  double e_cg = 0.0;
  double x = 0.0;
  double omega = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double last_time_x = 0.;
  double last_time_theta = 0.;

  std::vector<double> filter_vx;
  std::vector<double> filter_vtheta;
  std::vector<double> filter_x;
  std::vector<double> filter_theta;
  Eigen::MatrixXd X;
  Eigen::MatrixXd x0;
  Eigen::MatrixXd P;
  Eigen::MatrixXd P0;
  Eigen::MatrixXd PATH;
  Eigen::MatrixXd ecg_time_vec;
  Eigen::MatrixXd theta_e_time_vec;
  Eigen::MatrixXd state_est;
  Eigen::MatrixXd state_measurement;
  Eigen::MatrixXd xp;
  Eigen::MatrixXd yp;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;
  LqgControllerConfig::SharedPtr config_;
  LqgState::SharedPtr state_;
  LqrCalculator art_lqr;
  LkfCalculator art_lkf;

  void update_states();
  double delta_constraints(double steering_angle);
};

}  // namespace race

#endif  // LQG_CONTROLLER__LQG_CONTROLLER_HPP_
