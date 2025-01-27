// Copyright 2023 AI Racing Tech
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
 * @file dynamic_bicycle_model.hpp
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @author Dominic Nightingale (djnighti@ucsd.edu)
 * @brief
 * @version 0.2
 * @date 2023-05-04
 *
 * @copyright AI Racing Tech 2023
 *
 */

#ifndef LQG_CONTROLLER__DYNAMIC_BICYCLE_MODEL_HPP_
#define LQG_CONTROLLER__DYNAMIC_BICYCLE_MODEL_HPP_

#include <Eigen/Core>
#include <math.h>
#include <vector>
#include <memory>
#include <iostream>

#define GRAVITY_CONST 9.81
namespace race
{

struct DynamicBicycleModelConfig
{
  typedef std::shared_ptr<DynamicBicycleModelConfig> SharedPtr;
  double m;
  double lf;
  double lr;
  double cf;
  double cr;
  double iz;
  double ts;
};

class DynamicBicycleModel
{
public:
  double m;
  double lf;
  double lr;
  double cf;
  double cr;
  double iz;
  double ts;
  typedef std::shared_ptr<DynamicBicycleModel> SharedPtr;
  Eigen::MatrixXd A, B, C, D;
  explicit DynamicBicycleModel(DynamicBicycleModelConfig::SharedPtr config)
  {
    m = config->m;
    lf = config->lf;
    lr = config->lr;
    cf = config->cf;
    cr = config->cr;
    iz = config->iz;
    ts = config->ts;

    A = Eigen::MatrixXd(4, 4);
    B = Eigen::MatrixXd(4, 2);
    C = Eigen::MatrixXd(4, 4);
    D = Eigen::MatrixXd::Zero(2, 4);
  }

  void create_car_model(double Vx, double curvature, double bank_angle)
  {
    // Build CT model with updated longitudinal velocity (Vx)
    this->create_ct_ss_model(Vx, curvature, bank_angle);
    // cout << this->A << endl;
    // Convert to DT
    this->c2d();
    // cout << this->A << endl;
  }

private:
  // double ts;

  void create_ct_ss_model(double Vx, double curvature, double bank_angle)
  {
    // Create continous time state space model
    // double curvature = 0.;
    // Dynamics matrix (A) 4x4
    if (curvature == 0.) {curvature = 0.001;}
    // cout << Vx << endl;
    this->A(0, 0) = 0;
    this->A(0, 1) = 1;
    this->A(0, 2) = 0;
    this->A(0, 3) = 0;

    this->A(1, 0) = 0;
    this->A(1, 1) = -(cf + cr) / (m * Vx);
    this->A(1, 2) = (cf + cr) / m;
    this->A(1, 3) = -(lf * cf - lr * cr) / (m * Vx);

    this->A(2, 0) = 0;
    this->A(2, 1) = 0;
    this->A(2, 2) = 0;
    this->A(2, 3) = 1;

    this->A(3, 0) = 0;
    this->A(3, 1) = -(lf * cf - lr * cr) / (iz * Vx);
    this->A(3, 2) = (lf * cf - lr * cr) / iz;
    this->A(3, 3) = -(lf * lf * cf + lr * lr * cr) / (iz * Vx);

    // Input matrix (B) 4x2
    this->B(0, 0) = 0;
    this->B(1, 0) = cf / m;
    this->B(2, 0) = 0;
    this->B(3, 0) = lf * cf / iz;

    // cout << "Added steering : " << lf * curvature << endl;
    this->D(0, 0) = 0;
    this->D(1, 0) = set_e_theta_ss(Vx, lr, lf, m, curvature, bank_angle);
    this->D(2, 0) = 0;
    this->D(3, 0) = set_delta_ff(Vx, lr, lf, m, curvature);  // (lr+lf)/(curvature);

    // Measurement matrix (C) 2x4
    this->C(0, 0) = 1;
    this->C(0, 1) = 0;
    this->C(0, 2) = 0;
    this->C(0, 3) = 0;

    this->C(1, 0) = 0;
    this->C(1, 1) = 1;
    this->C(1, 2) = 0;
    this->C(1, 3) = 0;

    this->C(2, 0) = 0;
    this->C(2, 1) = 0;
    this->C(2, 2) = 1;
    this->C(2, 3) = 0;

    this->C(3, 0) = 0;
    this->C(3, 1) = 0;
    this->C(3, 2) = 0;
    this->C(3, 3) = 1;
  }

  double set_e_theta_ss(
    double Vx, double lr, double lf, double m, double curvature,
    double bank_angle)
  {
    double l = lr + lf;
    // cout << "Bank angle : " << bank_angle << endl;
    double ay = pow(Vx, 2) * curvature + GRAVITY_CONST * sin(bank_angle);
    // double Kv = (lr * m / (cf * l)) - (lf * m / (cr * l));
    double e_theta_ss = -lr * curvature;  // + (lf * m * ay / (2. * cr * l));
    return e_theta_ss;
  }

  double set_delta_ff(double Vx, double lr, double lf, double m, double curvature)
  {
    double l = lr + lf;
    // double ay = pow(Vx, 2) * curvature;
    // double Kv = (lr * m / (cf * l)) - (lf * m / (cr * l));
    // double e_theta_ss = -lr * curvature + (lf * m * ay / (cr * l));
    double d_ff_1 = l * curvature;
    // double d_ff_2 = Kv * ay;
    double d_ff = d_ff_1;
    return d_ff;
  }

  void c2d()
  {
    // Transform continous time state space model to discrete time
    // FIXME! to size of inputs A and B
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(4, 2);
    Eigen::MatrixXd A_Ts_k;
    int k_fac;
    int k_1_fac;
    int max_itt = 10;

    for (int k = 0; k < max_itt; k++) {
      k_fac = factorial(k);
      k_1_fac = factorial(k + 1);
      A_Ts_k = pow2(A * ts, k);
      // cout << "haha" << A_Ts_k << endl;
      // A matrix
      Ad = Ad + (A_Ts_k) / k_fac;

      // B matrix
      Bd = Bd + (A_Ts_k * this->ts / k_1_fac) * this->B;
    }
    this->A = Ad;
    this->B = Bd;
  }

  int factorial(int step)
  {
    // factorial math operator
    int result = 1;
    for (int k = 2; k <= step; k++) {
      result = k * result;
    }
    return result;
  }

  // TODO(haoru): use built-in Eigen routine
  Eigen::MatrixXd pow2(Eigen::MatrixXd A, int k)
  {
    Eigen::MatrixXd res = Eigen::MatrixXd::Identity(4, 4);
    for (int i = 0; i < k; i++) {
      res = res * A;
    }
    return res;
  }
};
}  // namespace race
#endif  // LQG_CONTROLLER__DYNAMIC_BICYCLE_MODEL_HPP_
