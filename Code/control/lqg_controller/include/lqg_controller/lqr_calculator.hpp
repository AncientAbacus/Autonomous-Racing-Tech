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
 * @file lqr_calculator.hpp
 * @author Dominic Nightingale (djnighti@ucsd.edu)
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @brief
 * @version 0.2
 * @date 2022-7-15
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#ifndef LQG_CONTROLLER__LQR_CALCULATOR_HPP_
#define LQG_CONTROLLER__LQR_CALCULATOR_HPP_

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include "Eigen/Core"
#include "rclcpp/rclcpp.hpp"
namespace race
{

struct StatePerformance
{
  double l_min;
  double l_factor;
  double l_const;
  double rc_1_val;
};

struct LqrCalculatorConfig
{
  typedef std::shared_ptr<LqrCalculatorConfig> SharedPtr;
  double are_max_iter;
  double are_error_thresh;
  // double rc_1_val;
  double Ts;
  double Vmin, Vmax;
  StatePerformance state_performance;
};

class LqrCalculator
{
public:
  double qc_1_a;
  double qc_1_b;
  double qc_1_c;
  double qc_2_a;
  double qc_2_b;
  double qc_2_c;
  double qc_3_a;
  double qc_3_b;
  double qc_3_c;
  double qc_4_a;
  double qc_4_b;
  double qc_4_c;
  typedef std::shared_ptr<LqrCalculator> SharedPtr;
  explicit LqrCalculator(LqrCalculatorConfig::SharedPtr config);
  Eigen::MatrixXd lqr_step(double vx, Eigen::MatrixXd A, Eigen::MatrixXd B);
  Eigen::MatrixXd compute_a_powerfit_coeff(Eigen::MatrixXd x, Eigen::MatrixXd y);
  void compute_all_powerfit_coeff();
  void recalculate_params();
  LqrCalculatorConfig::SharedPtr get_config();

private:
  LqrCalculatorConfig::SharedPtr config_;
  Eigen::MatrixXd Qc, q_range, V_range;
  Eigen::MatrixXd Rc, qc_1_data, qc_2_data, qc_3_data, qc_4_data;
  double qc_1_coeff, qc_3_coeff, rc_1_val, Ts, Vmin, Vmax, are_error_thresh, are_max_iter;
  // int are_max_iter;
  double qc_1_val = 0.0;
  double qc_2_val = 0.0;
  double qc_3_val = 0.0;
  double qc_4_val = 0.0;
  double l_factor = 0.;
  double l_const = 0.;
  double ld_min = 0.;
  double power_function_eval(double vx, double a, double b);
  void compute_weights(double vx);
};

}  // namespace race
#endif  // LQG_CONTROLLER__LQR_CALCULATOR_HPP_
