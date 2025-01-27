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

#include <string>
#include <memory>
#include <vector>

#include "vehicle_model/ros_param_loader.hpp"

namespace race
{
namespace vehicle_model
{
void load_parameters(rclcpp::Node * node, VehicleModel & model)
{
  auto declare_double = [&](const char * name) {
      try {
        return node->declare_parameter<double>(name);
      } catch (std::exception & e) {
        RCLCPP_INFO(
          node->get_logger(), "Exception thrown when declaring parameter \"%s\": %s", name,
          e.what());
        throw e;
      }
    };
  auto declare_vec = [&](const char * name) {
      try {
        return node->declare_parameter<std::vector<double>>(name);
      } catch (std::exception & e) {
        RCLCPP_INFO(
          node->get_logger(), "Exception thrown when declaring parameter \"%s\": %s", name,
          e.what());
        throw e;
      }
    };

  const auto front_normal_load = declare_vec("front_tyre.lookup_per_tyre.normal_load");
  auto front_tyre_config = std::make_shared<TyreConfig>(
    TyreConfig{
        declare_double("front_tyre.radius"),
        declare_double("front_tyre.width"),
        declare_double("front_tyre.mass"),
        declare_double("front_tyre.rot"),
        declare_double("front_tyre.rolling_resistance_coeff"),
        declare_double("front_tyre.pacejka_b"),
        declare_double("front_tyre.pacejka_c"),
        declare_double("front_tyre.pacejka_e"),
        declare_double("front_tyre.pacejka_fz0"),
        declare_double("front_tyre.pacejka_eps"),
        Lookup2D{
          front_normal_load,
          declare_vec("front_tyre.lookup_per_tyre.max_lon_slip"),
        },
        Lookup2D{
          front_normal_load,
          declare_vec("front_tyre.lookup_per_tyre.max_lat_deg"),
        },
        Lookup2D{
          front_normal_load,
          declare_vec("front_tyre.lookup_per_tyre.max_lon_force"),
        },
        Lookup2D{
          front_normal_load,
          declare_vec("front_tyre.lookup_per_tyre.max_lat_force"),
        }
      }
  );

  const auto rear_normal_load = declare_vec("rear_tyre.lookup_per_tyre.normal_load");
  auto rear_tyre_config = std::make_shared<TyreConfig>(
    TyreConfig{
        declare_double("rear_tyre.radius"),
        declare_double("rear_tyre.width"),
        declare_double("rear_tyre.mass"),
        declare_double("rear_tyre.rot"),
        declare_double("rear_tyre.rolling_resistance_coeff"),
        declare_double("rear_tyre.pacejka_b"),
        declare_double("rear_tyre.pacejka_c"),
        declare_double("rear_tyre.pacejka_e"),
        declare_double("rear_tyre.pacejka_fz0"),
        declare_double("rear_tyre.pacejka_eps"),
        Lookup2D{
          rear_normal_load,
          declare_vec("rear_tyre.lookup_per_tyre.max_lon_slip"),
        },
        Lookup2D{
          rear_normal_load,
          declare_vec("rear_tyre.lookup_per_tyre.max_lat_deg"),
        },
        Lookup2D{
          rear_normal_load,
          declare_vec("rear_tyre.lookup_per_tyre.max_lon_force"),
        },
        Lookup2D{
          rear_normal_load,
          declare_vec("rear_tyre.lookup_per_tyre.max_lat_force"),
        }
      }
  );

  auto front_brake_config = std::make_shared<BrakeConfig>(
    BrakeConfig{
        declare_double("front_brake.max_brake"),
        declare_double("front_brake.brake_pad_out_r"),
        declare_double("front_brake.brake_pad_in_r"),
        declare_double("front_brake.brake_caliper_area"),
        declare_double("front_brake.brake_pad_friction_coeff"),
        declare_double("front_brake.piston_area"),
        declare_double("front_brake.bias")
      }
  );

  auto rear_brake_config = std::make_shared<BrakeConfig>(
    BrakeConfig{
        declare_double("rear_brake.max_brake"),
        declare_double("rear_brake.brake_pad_out_r"),
        declare_double("rear_brake.brake_pad_in_r"),
        declare_double("rear_brake.brake_caliper_area"),
        declare_double("rear_brake.brake_pad_friction_coeff"),
        declare_double("rear_brake.piston_area"),
        declare_double("rear_brake.bias")
      }
  );

  auto steer_config = std::make_shared<SteerConfig>(
    SteerConfig{
        declare_double("steer.max_steer_rate"),
        declare_double("steer.max_steer"),
        declare_double("steer.turn_left_bias")
      }
  );

  auto chassis_config = std::make_shared<ChassisConfig>(
    ChassisConfig{
        declare_double("chassis.total_mass"),
        declare_double("chassis.sprung_mass"),
        declare_double("chassis.unsprung_mass"),
        declare_double("chassis.cg_ratio"),
        declare_double("chassis.wheel_base"),
        declare_double("chassis.track_width"),
        declare_double("chassis.moi")
      }
  );

  auto aero_config = std::make_shared<AeroConfig>(
    AeroConfig{
        declare_double("aero.aero_ratio"),
        Lookup2D{
          declare_vec("aero.speed"),
          declare_vec("aero.downforce")
        },
        declare_double("aero.air_density"),
        declare_double("aero.drag_coeff"),
        declare_double("aero.cross_area")
      }
  );

  auto powertrain_config = std::make_shared<PowerTrainConfig>(
    PowerTrainConfig{
        Lookup3D{
          declare_vec("powertrain.rpm"),
          declare_vec("powertrain.throttle"),
          declare_vec("powertrain.torque")
        },
        declare_vec("powertrain.gear_ratio"),
        declare_double("powertrain.final_drive_ratio"),
        declare_double("powertrain.sample_throttle")
      }
  );

  auto vehicle_model_config = std::make_shared<VehicleModelConfig>(
    VehicleModelConfig{
        front_tyre_config,
        rear_tyre_config,
        front_brake_config,
        rear_brake_config,
        steer_config,
        chassis_config,
        aero_config,
        powertrain_config
      }
  );
  model.set_config(vehicle_model_config);
}
}  // namespace vehicle_model
}  // namespace race
