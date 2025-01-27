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
 * @file lkf_calculator.hpp
 * @author Dominic Nightingale (djnighti@ucsd.edu)
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @brief
 * @version 0.2
 * @date 2022-7-15
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#ifndef LQG_CONTROLLER__LKF_CALCULATOR_HPP_
#define LQG_CONTROLLER__LKF_CALCULATOR_HPP_

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include "Eigen/Core"

namespace race
{

struct ModelError
{
  double qf_1;
  double qf_2;
  double qf_3;
  double qf_4;
};

struct MeasurementError
{
  double rf_1;
  double rf_2;
  double rf_3;
  double rf_4;
};

struct LkfCalculatorConfig
{
  typedef std::shared_ptr<LkfCalculatorConfig> SharedPtr;
  ModelError model_error;
  MeasurementError measurement_error;
};

class LkfCalculator
{
public:
  typedef std::shared_ptr<LkfCalculator> SharedPtr;
  explicit LkfCalculator(LkfCalculatorConfig::SharedPtr config);
  void lkf_step(
    Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
    Eigen::MatrixXd & xhat, Eigen::MatrixXd u, Eigen::MatrixXd y, Eigen::MatrixXd & P);

private:
  double ts;
  LkfCalculatorConfig::SharedPtr config_;
  Eigen::MatrixXd Qf;
  Eigen::MatrixXd Rf;

  void compute_weights();
};
}  // namespace race
#endif  // LQG_CONTROLLER__LKF_CALCULATOR_HPP_
