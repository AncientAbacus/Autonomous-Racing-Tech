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

#ifndef VEHICLE_MODEL__VEHICLE_MODEL_HPP_
#define VEHICLE_MODEL__VEHICLE_MODEL_HPP_

#include <memory>
#include <string>

#include "vehicle_model/vehicle_model_config.hpp"

namespace race
{
namespace vehicle_model
{
struct LonControlOutput
{
  double throttle_level;
  double brake_psi;
  double slip_angle_rad;
};

struct LatControlOutput
{
  double cg_slip_angle_rad;  // modeling result of the current slip angle
  double front_slip_angle_rad;  // modeling result of the current slip angle
  double rear_slip_angle_rad;  // modeling result of the current slip angle
  double target_front_slip_angle_rad;  // the target slip angle for the target lat acc
};

struct VehicleModelState
{
  double speed_mps;
  double slip_angle;
  double steer_radius;
  double lon_acc;
  double lat_acc;
  double yaw_rate;
  double steer_angle;
  double bank_angle;
  double engine_rpm;
  size_t gear_num;
};

class VehicleModel
{
public:
  typedef std::shared_ptr<VehicleModel> SharedPtr;
  typedef std::unique_ptr<VehicleModel> UniquePtr;
  explicit VehicleModel(VehicleModelConfig::SharedPtr config);
  void set_config(VehicleModelConfig::SharedPtr config);
  const VehicleModelConfig & get_config() const;

  LonControlOutput calc_lon_control(const VehicleModelState & state, const double & target_acc);
  LatControlOutput calc_lat_control(const VehicleModelState & state, const double & target_acc);
  double get_available_lon_acc(const VehicleModelState & state);
  double get_available_lat_acc(const VehicleModelState & state);
  double calc_resistance(const double & current_speed, const double & bank_angle);
  double calc_ax(const double & throttle, const VehicleModelState & state);
  void calc_norm_force(
    const double & current_speed, const double & bank_angle, double & front_df,
    double & rear_df);

private:
  double calc_net_force(const double & target_acc);
  double calc_brake(const double & control_force);
  double calc_throttle(
    const double & engine_rpm, const size_t & gear_num,
    const double & control_force);
  void lookup_traction_circle_per_tyre(
    const double & normal_force, const Lookup2D & lon_lookup,
    const Lookup2D lat_lookup, double & max_lon_force,
    double & max_lat_force);
  void calc_max_lon_lat_forces(
    const VehicleModelState & state, double & max_front_lon,
    double & max_front_lat, double & max_rear_lon,
    double & max_rear_lat);
  VehicleModelConfig::SharedPtr config_ {};
};
}  // namespace vehicle_model
}  // namespace race
#endif  // VEHICLE_MODEL__VEHICLE_MODEL_HPP_
