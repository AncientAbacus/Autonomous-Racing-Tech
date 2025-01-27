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

#include <Eigen/Dense>
#include <math.h>
#include <algorithm>
#include <string>
#include <memory>
#include <iostream>

#include "vehicle_model/vehicle_model.hpp"
#include "vehicle_model/vehicle_model_config.hpp"
#include "vehicle_model/lookup.hpp"

#define GRAVITY 9.81
namespace race
{
namespace vehicle_model
{
VehicleModel::VehicleModel(VehicleModelConfig::SharedPtr config = {})
: config_(config)
{
}

void VehicleModel::set_config(VehicleModelConfig::SharedPtr config)
{
  config_ = config;
}

const VehicleModelConfig & VehicleModel::get_config() const
{
  return *config_.get();
}

LonControlOutput VehicleModel::calc_lon_control(
  const VehicleModelState & state,
  const double & target_acc)
{
  try {
    const auto limited_target_acc =
      std::min(abs(target_acc), get_available_lon_acc(state) / 2) * (target_acc > 0 ? 1.0 : -1.0);
    const auto net_force = calc_net_force(limited_target_acc);
    const auto resistance = calc_resistance(state.speed_mps, state.bank_angle);
    // control_f + resistance_f = net_f
    const auto control_force = net_force - resistance;

    const auto target_wheel_torque = control_force * config_->rear_tyre_config->radius;
    const auto target_engine_torque = target_wheel_torque /
      (config_->powertrain_config->gear_ratio[state.gear_num - 1] *
      config_->powertrain_config->final_drive_ratio);
    const auto min_engine_torque = bilinear_interpolate(
      config_->powertrain_config->torque_v_rpm_throttle, state.engine_rpm, 0.0, false);

    const auto min_control_force = min_engine_torque *
      (config_->powertrain_config->gear_ratio[state.gear_num - 1] *
      config_->powertrain_config->final_drive_ratio) / config_->rear_tyre_config->radius;
    // if (control_force >= 0) {
    if (target_engine_torque >= min_engine_torque) {
      // If control force is possitive, throttle up.
      return LonControlOutput{calc_throttle(state.engine_rpm, state.gear_num, control_force), 0.0,
        0.0};
    } else {
      // If control force is negative, brake.
      return LonControlOutput{0.0, calc_brake(control_force - min_control_force), 0.0};
    }
  } catch (const std::exception & e) {
    printf("%s", e.what());
  }
  return LonControlOutput{0.0, 0.0, 0.0};
}

LatControlOutput VehicleModel::calc_lat_control(
  const VehicleModelState & state,
  const double & target_acc)
{
  // calculate the required forces
  const double limited_target_acc = std::min(target_acc, get_available_lat_acc(state));
  double front_normal_force = 0.0, rear_normal_force = 0.0;
  calc_norm_force(state.speed_mps, state.bank_angle, front_normal_force, rear_normal_force);
  const double max_front_lat_force = linear_interpolate(
    config_->front_tyre_config->max_lat_force,
    front_normal_force, false);
  const double max_rear_lat_force = linear_interpolate(
    config_->rear_tyre_config->max_lat_force,
    rear_normal_force, false);
  const double max_front_slip_angle = linear_interpolate(
    config_->front_tyre_config->max_lat_deg,
    front_normal_force, false) * M_PI / 180.0;
  const double max_rear_slip_angle = linear_interpolate(
    config_->rear_tyre_config->max_lat_deg,
    rear_normal_force, false) * M_PI / 180.0;

  // get linearlized cornering stiffness coefficient
  const double cf = max_front_lat_force / max_front_slip_angle;
  const double cr = max_rear_lat_force / max_rear_slip_angle;

  // calculate cg
  const double & cg_ratio = config_->chassis_config->cg_ratio;
  const double cg_to_rear = cg_ratio * config_->chassis_config->wheel_base;
  const double cg_to_front = config_->chassis_config->wheel_base - cg_to_rear;

  // estimate slip angle
  const double a_22 = -(2 * cf + 2 * cr) / (config_->chassis_config->total_mass * state.speed_mps);
  // const double a_42 = -(2 * cf * cg_to_front - 2 * cr * cg_to_rear) /
  //   config_->chassis_config->moi;
  // const double b_41 = 2 * cf * cg_to_front / config_->chassis_config->moi;
  const double a_23 = -a_22 * state.speed_mps;
  const double a_24 = -1 - (2 * cf * cg_to_front - 2 * cr * cg_to_rear) /
    (config_->chassis_config->total_mass * pow(state.speed_mps, 2));
  const double b_21 = (2 * cf ) / (config_->chassis_config->total_mass * state.speed_mps);
  const double b_p_21 = b_21 * state.speed_mps;
  const double a_y = state.lat_acc + state.speed_mps * state.yaw_rate - GRAVITY * sin(
    state.bank_angle);

  // Eigen::Vector3d x(state.slip_angle, state.slip_angle, state.yaw_rate);
  // Eigen::Matrix3d A;
  // A << 0.1, 0.0, 0.0,
  //   0.0, 0.0, 1.0,
  //   0.0, -a_42, a_42;
  // Eigen::Vector3d B_delta(0.0, 0.0, b_41);
  // Eigen::MatrixXd B_alpha_phi(3, 2);
  // B_alpha_phi << 0.0, 0.0,
  //   -1.0 / state.speed_mps, -GRAVITY / state.speed_mps,
  //   0.0, 0.0;
  // Eigen::Vector2d b(a_y, sin(state.bank_angle));
  // Eigen::Vector3d x_dot = A * x + B_delta * state.steer_angle + B_alpha_phi * b;
  // LatControlOutput output;
  // output.rear_slip_angle_rad = -x_dot(0) - cg_to_rear * state.yaw_rate / state.speed_mps;
  // output.front_slip_angle_rad = -x_dot(0) + cg_to_front * state.yaw_rate / state.speed_mps
  //    - state.steer_angle;
  // output.cg_slip_angle_rad = x_dot(0);
  //  std::cout << x_dot(0) << " " << x_dot(1) << " " << x_dot(2) << std::endl;

  const double beta =
    (a_y - (1 + a_24) * state.speed_mps * state.yaw_rate - b_p_21 * state.steer_angle) / a_23;
  LatControlOutput output;
  output.rear_slip_angle_rad = 0.0 - (-beta - cg_to_rear * state.yaw_rate / state.speed_mps);
  output.front_slip_angle_rad = state.steer_angle -
    (-beta + cg_to_front * state.yaw_rate / state.speed_mps);
  output.cg_slip_angle_rad = beta;
  // std::cout << output.cg_slip_angle_rad << " "
  //   << output.front_slip_angle_rad << " " << output.rear_slip_angle_rad << std::endl;

  // calculate control targets
  const double net_force = calc_net_force(limited_target_acc);
  const double rear_force = output.rear_slip_angle_rad * cr;
  const double front_force = net_force - rear_force;
  output.target_front_slip_angle_rad = std::clamp(
    front_force / cf, -max_front_slip_angle,
    max_front_slip_angle);
  // std::cout << net_force << " " << rear_force << " " << front_force
  //   << " " << output.target_front_slip_angle_rad << std::endl;

  return output;
}

double VehicleModel::get_available_lon_acc(const VehicleModelState & state)
{
  double max_front_lat_force = 0.0, max_front_lon_force = 0.0, max_rear_lat_force = 0.0,
    max_rear_lon_force = 0.0;
  calc_max_lon_lat_forces(
    state, max_front_lon_force, max_front_lat_force, max_rear_lon_force,
    max_rear_lat_force);

  // m * g * sin(bank) + f_fric = m * a_lat
  const auto lat_force = config_->chassis_config->total_mass *
    (state.lat_acc + GRAVITY * sin(state.bank_angle));
  const auto front_lat_force = lat_force * config_->chassis_config->cg_ratio;
  const auto rear_lat_force = lat_force * (1 - config_->chassis_config->cg_ratio);

  // Don't accelerate if already breaking the traction circle
  if (front_lat_force > max_front_lat_force || rear_lat_force > max_rear_lat_force) {
    return 0.0;
  }

  // Find max lon acc on a diamond traction circle model
  const auto front_lon_force = max_front_lon_force - front_lat_force / max_front_lat_force *
    max_front_lon_force;
  const auto rear_lon_force = max_rear_lon_force - rear_lat_force / max_rear_lat_force *
    max_rear_lon_force;
  const auto lon_force = std::min(front_lon_force, rear_lon_force) * 2;
  return lon_force / config_->chassis_config->total_mass;
}

double VehicleModel::get_available_lat_acc(const VehicleModelState & state)
{
  double max_front_lat_force = 0.0, max_front_lon_force = 0.0, max_rear_lat_force = 0.0,
    max_rear_lon_force = 0.0;
  calc_max_lon_lat_forces(
    state, max_front_lon_force, max_front_lat_force, max_rear_lon_force,
    max_rear_lat_force);

  // f_lon = m * a
  const auto lon_force = config_->chassis_config->total_mass * state.lon_acc;
  const auto front_lon_force = lon_force * config_->chassis_config->cg_ratio;
  const auto rear_lon_force = lon_force * (1 - config_->chassis_config->cg_ratio);

  // Don't accelerate if already breaking the traction circle
  if (abs(front_lon_force) > max_front_lon_force || abs(rear_lon_force) > max_rear_lon_force) {
    return 0.0;
  }

  // Find max lat acc on a diamond traction circle model
  const auto front_lat_force = max_front_lat_force - front_lon_force / max_front_lon_force *
    max_front_lat_force;
  const auto rear_lat_force = max_rear_lat_force - rear_lon_force / max_rear_lon_force *
    max_rear_lat_force;
  const auto lat_force = std::min(front_lat_force, rear_lat_force) * 2;
  return lat_force / config_->chassis_config->total_mass;
}

double VehicleModel::calc_net_force(const double & target_acc)
{
  // F = m * a
  return target_acc * config_->chassis_config->total_mass;
}

void VehicleModel::calc_norm_force(
  const double & current_speed, const double & bank_angle, double & front_df,
  double & rear_df)
{
  const auto total_downforce = linear_interpolate(
    config_->aero_config->downforce_v_speed,
    current_speed, false);
  const auto front_downforce = total_downforce * config_->aero_config->aero_ratio;
  const auto rear_downforce = total_downforce - front_downforce;

  front_df = config_->chassis_config->total_mass *
    config_->chassis_config->cg_ratio * GRAVITY * cos(bank_angle) + front_downforce;
  rear_df = config_->chassis_config->total_mass *
    (1 - config_->chassis_config->cg_ratio) * GRAVITY * cos(bank_angle) + rear_downforce;
}

double VehicleModel::calc_resistance(const double & current_speed, const double & bank_angle)
{
  double f_front_norm = 0.0, f_rear_norm = 0.0;
  calc_norm_force(current_speed, bank_angle, f_front_norm, f_rear_norm);
  // std::cout << f_front_norm << " " << f_rear_norm << std::endl;

  const auto front_rolling_resistance = config_->front_tyre_config->rolling_resistance_coeff *
    f_front_norm;
  const auto rear_rolling_resistance = config_->rear_tyre_config->rolling_resistance_coeff *
    f_rear_norm;

  // F_air = 0.5 * air_density * v^2 * drag_coeff * area
  const auto air_resistance = 0.5 * config_->aero_config->air_density * current_speed *
    current_speed *
    config_->aero_config->drag_coeff * config_->aero_config->cross_area;

  return -1.0 * (air_resistance + front_rolling_resistance + rear_rolling_resistance);
}

double VehicleModel::calc_brake(const double & control_force)
{
  if (control_force >= 0.0) {
    return 0.0;
  }
  const auto front_control_torque = config_->front_brake_config->bias * control_force *
    config_->front_tyre_config->radius;
  const auto rear_control_torque = config_->rear_brake_config->bias * control_force *
    config_->rear_tyre_config->radius;
  const auto front_brake_lever =
    (config_->front_brake_config->brake_pad_in_r + config_->front_brake_config->brake_pad_out_r) /
    2.0;
  const auto rear_brake_lever =
    (config_->rear_brake_config->brake_pad_in_r + config_->rear_brake_config->brake_pad_out_r) /
    2.0;

  // T = lever * friction_mu * pressure * pi * piston_radius^2 * num_pad
  const auto front_brake_kpa = -0.001 * front_control_torque /
    (front_brake_lever * config_->front_brake_config->brake_pad_friction_coeff *
    config_->front_brake_config->piston_area * 2);
  const auto rear_brake_kpa = -0.001 * rear_control_torque /
    (rear_brake_lever * config_->rear_brake_config->brake_pad_friction_coeff *
    config_->rear_brake_config->piston_area * 2);

  const auto ave_brake_kpa =
    (std::clamp(
      front_brake_kpa, 0.0,
      config_->front_brake_config->max_brake) +
    std::clamp(rear_brake_kpa, 0.0, config_->rear_brake_config->max_brake)) / 2.0;
  return ave_brake_kpa;
}

double VehicleModel::calc_ax(const double & throttle, const VehicleModelState & state)
{
  if (state.gear_num > config_->powertrain_config->gear_ratio.size() || state.gear_num < 1) {
    printf("Gear number of %lu is not possible.", state.gear_num);
    return 0.0;
  }
  const auto sample_engine_torque = bilinear_interpolate(
    config_->powertrain_config->torque_v_rpm_throttle, state.engine_rpm, throttle, false);
  const auto wheel_torque = sample_engine_torque *
    (config_->powertrain_config->gear_ratio[state.gear_num - 1] *
    config_->powertrain_config->final_drive_ratio);
  const auto force_wheels = wheel_torque / config_->rear_tyre_config->radius;
  const auto resistance = calc_resistance(state.speed_mps, state.bank_angle);
  const auto net_force = force_wheels + resistance;
  return net_force / config_->chassis_config->total_mass;
}

double VehicleModel::calc_throttle(
  const double & engine_rpm, const size_t & gear_num,
  const double & control_force)
{
  // if (control_force <= 0.0) {
  //   return 0.0;
  // }
  if (gear_num > config_->powertrain_config->gear_ratio.size() || gear_num < 1) {
    printf("Gear number of %lu is not possible.", gear_num);
    return 0.0;
  }

  const auto target_wheel_torque = control_force * config_->rear_tyre_config->radius;
  const auto target_engine_torque = target_wheel_torque /
    (config_->powertrain_config->gear_ratio[gear_num - 1] *
    config_->powertrain_config->final_drive_ratio);
  const auto min_engine_torque = bilinear_interpolate(
    config_->powertrain_config->torque_v_rpm_throttle, engine_rpm, 0.0, false);
  const auto max_engine_torque = bilinear_interpolate(
    config_->powertrain_config->torque_v_rpm_throttle, engine_rpm, 100.0, false);
  const auto sample_throttle = config_->powertrain_config->sample_throttle;
  const auto sample_engine_torque = bilinear_interpolate(
    config_->powertrain_config->torque_v_rpm_throttle, engine_rpm, sample_throttle, false);
  if (target_engine_torque <= sample_engine_torque) {
    return fast_linear_interpolate(
      min_engine_torque, sample_engine_torque, 0.0, sample_throttle,
      target_engine_torque, false);
  } else {
    return fast_linear_interpolate(
      sample_engine_torque, max_engine_torque, sample_throttle, 100.0,
      target_engine_torque, false);
  }
}

void VehicleModel::lookup_traction_circle_per_tyre(
  const double & normal_force,
  const Lookup2D & lon_lookup,
  const Lookup2D lat_lookup,
  double & max_lon_force, double & max_lat_force)
{
  max_lon_force = linear_interpolate(lon_lookup, normal_force, false);
  max_lat_force = linear_interpolate(lat_lookup, normal_force, false);
}

void VehicleModel::calc_max_lon_lat_forces(
  const VehicleModelState & state, double & max_front_lon,
  double & max_front_lat, double & max_rear_lon,
  double & max_rear_lat)
{
  // look up the traction circle for limits
  double f_front_norm = 0.0, f_rear_norm = 0.0;
  calc_norm_force(state.speed_mps, state.bank_angle, f_front_norm, f_rear_norm);
  lookup_traction_circle_per_tyre(
    f_front_norm / 2.0, config_->front_tyre_config->max_lon_force,
    config_->front_tyre_config->max_lat_force, max_front_lon,
    max_front_lat);
  lookup_traction_circle_per_tyre(
    f_rear_norm / 2.0, config_->rear_tyre_config->max_lon_force,
    config_->rear_tyre_config->max_lat_force, max_rear_lon,
    max_rear_lat);
  // times 2 since it is per tyre
  max_front_lat *= 2.0;
  max_front_lon *= 2.0;
  max_rear_lat *= 2.0;
  max_rear_lon *= 2.0;
}
}  // namespace vehicle_model
}  // namespace race
