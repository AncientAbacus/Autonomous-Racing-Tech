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

#ifndef VEHICLE_MODEL__VEHICLE_MODEL_CONFIG_HPP_
#define VEHICLE_MODEL__VEHICLE_MODEL_CONFIG_HPP_

#include <memory>
#include <vector>
#include "vehicle_model/lookup.hpp"

namespace race
{
namespace vehicle_model
{
struct TyreConfig
{
  typedef std::shared_ptr<TyreConfig> SharedPtr;
  double radius;  // m
  double width;  // m
  double mass;  // kg
  double rot;  // rotational moment of inertia in kg * m^2
  double rolling_resistance_coeff;

  // A more sophisticated model use Pacejka model
  double pacejka_b;  // Pacejka magic number
  double pacejka_c;  // Pacejka magic number
  double pacejka_e;  // Pacejka magic number
  double pacejka_fz0;  // Nominal normal force (N)
  double pacejka_eps;  // Nominal normal force (N)

  // While a less sophisticated model would just use the lookup table
  Lookup2D max_lon_slip;  // vs. normal load
  Lookup2D max_lat_deg;  // vs. normal load
  Lookup2D max_lon_force;  // vs. normal load
  Lookup2D max_lat_force;  // vs. normal load
};

struct BrakeConfig
{
  typedef std::shared_ptr<BrakeConfig> SharedPtr;
  double max_brake;  // psi
  double brake_pad_out_r;  // brake pad outter radius (m)
  double brake_pad_in_r;  // brake pad inner radius (m)
  double brake_caliper_area;  // brake pad area m^2
  double brake_pad_friction_coeff;  // brake pad kinetic friction coefficient
  double piston_area;  // Brake piston total area m^2
  double bias;  // ratio of total brake force
};

struct SteerConfig
{
  typedef std::shared_ptr<SteerConfig> SharedPtr;
  double max_steer_rate;  // rad/s
  double max_steer;  // rad
  double turn_left_bias;  // rad
};

struct ChassisConfig
{
  typedef std::shared_ptr<ChassisConfig> SharedPtr;
  double total_mass;  // kg
  double sprung_mass;  // kg
  double unsprung_mass;  // kg
  double cg_ratio;  // ratio of car weight on front axle
  double wheel_base;  // m
  double track_width;  // m
  double moi;  // polar moment of inertia (kg * m^2)
};

struct AeroConfig
{
  typedef std::shared_ptr<AeroConfig> SharedPtr;
  double aero_ratio;  // ratio of total downforce on front axle

  // lookup of speed (m/s) and total downforce (N)
  Lookup2D downforce_v_speed;

  // drag
  double air_density;  // kg/m^3
  double drag_coeff;
  double cross_area;  // m^2
};

// TODO(haoru): add suspension config when upgrading to 6 DOF model

struct PowerTrainConfig
{
  typedef std::shared_ptr<PowerTrainConfig> SharedPtr;

  // lookup of torque (N*m) against RPM (rev/min) and throttle (0-100)
  Lookup3D torque_v_rpm_throttle;

  // Gear ratio of each gear (beginning at 1st)
  std::vector<double> gear_ratio;

  double final_drive_ratio;  // Differential final drive ratio

  double sample_throttle;  // Sample Throttle

  // TODO(haoru): add mechanical loss
};

struct VehicleModelConfig
{
  typedef std::shared_ptr<VehicleModelConfig> SharedPtr;

  TyreConfig::SharedPtr front_tyre_config;
  TyreConfig::SharedPtr rear_tyre_config;
  BrakeConfig::SharedPtr front_brake_config;
  BrakeConfig::SharedPtr rear_brake_config;
  SteerConfig::SharedPtr steer_config;
  ChassisConfig::SharedPtr chassis_config;
  AeroConfig::SharedPtr aero_config;
  PowerTrainConfig::SharedPtr powertrain_config;
};
}  // namespace vehicle_model
}  // namespace race
#endif  // VEHICLE_MODEL__VEHICLE_MODEL_CONFIG_HPP_
