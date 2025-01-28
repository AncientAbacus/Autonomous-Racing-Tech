// Copyright 2024 AI Racing Tech
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

#include <Eigen/Core>

#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"
#include "ghost_car/controller_plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "rclcpp/qos.hpp"

using race_msgs::msg::VehicleManualControlCommand;
namespace race::ghost_car
{
class ManualController : public GhostCarController
{
public:
  bool configure() override
  {
    // Get the starting point of the path
    node().declare_parameter<std::vector<int>>("target_ttl_indexes");
    node().declare_parameter<int>("initial_wp_index", 0);
    node().declare_parameter<std::vector<double>>("target_speeds");
    node().declare_parameter<int>("ghost_car_count", 1);
    node().declare_parameter<std::string>("ttl_dir");
    node().get_parameter("dt", dt_);
    ttl_tree_ = std::make_unique<race::ttl::TtlTree>(
      node().get_parameter(
        "ttl_dir").as_string().c_str());
    if (!ttl_tree_->is_ttl_index_valid(
        node().get_parameter("target_ttl_indexes").as_integer_array()
        [0]))
    {
      throw std::runtime_error("Invalid ttl index passed");
    }
    target_ttl_index_ =
      static_cast<race::ttl::TtlIndex>(node().get_parameter("target_ttl_indexes").as_integer_array()
      [0]);
    int initial_wp_index = node().get_parameter("initial_wp_index").as_int();

    if (initial_wp_index < 0 ||
      static_cast<uint32_t>(initial_wp_index) >=
      ttl_tree_->get_ttl(target_ttl_index_).header.loop)
    {
      throw std::runtime_error("Invalid initial TTL index passed");
    }
    latest_pos_ = ttl_tree_->get_ttl(target_ttl_index_).waypoints.at(
      initial_wp_index).location;

    wheelbase = model().get_config().chassis_config->wheel_base;
    max_steer = model().get_config().steer_config->max_steer;
    max_steer_rate = model().get_config().steer_config->max_steer_rate;
    max_speed = node().get_parameter("target_speeds").as_double_array()[0] / 2.237;
    // Create a QoS object with 'best_effort' reliability
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort();

    // Create the subscription using the QoS settings
    command_subscriber_ = node().create_subscription<VehicleManualControlCommand>(
      "/joystick/control_command",
      qos,
      std::bind(&ManualController::commandCallback, this, std::placeholders::_1)
    );

    vehicle_model_ = std::make_shared<SimModelDelaySteerAcc>(
      max_speed,   // vx_lim: High velocity limit(doesnt work for some reason)
      max_steer,  // steer_lim: Max steering angle (e.g., 90 degrees in radians)
      100.0,  // vx_rate_lim: High rate limit for velocity
      max_steer_rate,  // steer_rate_lim: High rate limit for steering
      wheelbase,     // wheelbase: Actual wheelbase of the vehicle
      dt_,    // dt: Time step
      0.0,     // acc_delay: No delay in acceleration
      0.0,    // acc_time_constant:
      0.0,     // steer_delay: No delay in steering
      0.0,    // steer_time_constant
      0.0    // steer_dead_band: No dead zone in steering
    );
    Eigen::VectorXd state = Eigen::VectorXd::Zero(vehicle_model_->getDimX());
    state(0) = latest_pos_.x;
    state(1) = latest_pos_.y;
    vehicle_model_->setState(state);
    last_update_time_ = node().now();
    return true;
  }

  void commandCallback(const race_msgs::msg::VehicleManualControlCommand::SharedPtr msg)
  {
    // Extract commands from the message and update the vehicle model
    steering_angle = msg->vehicle_control_command.steering_cmd;
    double accelerator_cmd = msg->vehicle_control_command.accelerator_cmd;
    double brake_cmd = msg->vehicle_control_command.brake_cmd;
    int gear_cmd = msg->vehicle_control_command.gear_cmd;
    double desired_acceleration = calc_acceleration(accelerator_cmd, brake_cmd, gear_cmd);

    // Prepare the input vector for the vehicle model
    Eigen::VectorXd input = Eigen::VectorXd::Zero(vehicle_model_->getDimU());
    input(0) = desired_acceleration;
    input(1) = steering_angle;
    // Update the vehicle model
    auto current_time = node().now();
    dt_ = (current_time - last_update_time_).seconds();
    vehicle_model_->updateEuler(dt_, input);
    last_update_time_ = current_time;
  }


  void updatePositionAndSpeed(
    race::ttl::Position & position, double & speed,
    double & yaw_angle, int index) override
  {
    (void)index;
    // Update the position and speed using the vehicle model's state
    position.x = vehicle_model_->getX();
    position.y = vehicle_model_->getY();
    if (vehicle_model_->getVx() >= max_speed) {
      speed = max_speed;
    } else {
      speed = vehicle_model_->getVx();
    }
    yaw_angle = vehicle_model_->getYaw();
  }

  double calc_brake_force(double brake_pressure_kpa)
  {
    // Convert kPa to Pa (1 kPa = 1000 Pa)
    double brake_pressure_pa = brake_pressure_kpa * 1000;
    // Calculate the control torque for front and rear brakes
    // Using the formula: T = pressure * piston_area * brake_lever
    double front_control_torque = brake_pressure_pa *
      model().get_config().front_brake_config->piston_area *
      ((model().get_config().front_brake_config->brake_pad_in_r +
      model().get_config().front_brake_config->brake_pad_out_r) / 2.0);
    double rear_control_torque = brake_pressure_pa *
      model().get_config().rear_brake_config->piston_area *
      ((model().get_config().rear_brake_config->brake_pad_in_r +
      model().get_config().rear_brake_config->brake_pad_out_r) / 2.0);
    // Convert control torque to control force
    double control_force = (front_control_torque + rear_control_torque) /
      (2.0 * model().get_config().rear_tyre_config->radius);

    // Return the control force
    return control_force;
  }

  // calculate the resistance force
  double calc_resistance(const double & current_speed)
  {
    // Normal forces due to vehicle weight
    double f_front_norm = model().get_config().chassis_config->total_mass * 9.81 *
      model().get_config().chassis_config->cg_ratio;
    double f_rear_norm = model().get_config().chassis_config->total_mass * 9.81 *
      (1 - model().get_config().chassis_config->cg_ratio);
    // Calculate rolling resistance for front and rear
    const auto front_rolling_resistance =
      model().get_config().front_tyre_config->rolling_resistance_coeff * f_front_norm;
    const auto rear_rolling_resistance =
      model().get_config().rear_tyre_config->rolling_resistance_coeff * f_rear_norm;

    // Calculate air resistance
    const auto air_resistance = 0.5 * model().get_config().aero_config->air_density * pow(
      current_speed, 2) *
      model().get_config().aero_config->drag_coeff * model().get_config().aero_config->cross_area;

    // Total resistance is the sum of rolling and air resistance
    return -1.0 * (front_rolling_resistance + rear_rolling_resistance + air_resistance);
  }

  double calc_engine_rpm(const double & vehicle_speed, const int & gear_num)
  {
    // Calculate the wheel speed in rad/s
    double wheel_speed = vehicle_speed / model().get_config().rear_tyre_config->radius;
    // Calculate the engine speed in rad/s
    double engine_speed = wheel_speed * model().get_config().powertrain_config->final_drive_ratio *
      model().get_config().powertrain_config->gear_ratio[gear_num];
    // Convert engine speed to RPM
    return engine_speed * 60 / (2 * M_PI);
  }

  double calc_throttle_force(
    const double & throttle_level, const double & engine_rpm,
    const int & gear_num)
  {
    // Calculate the engine torque using the throttle level and engine RPM
    if (gear_num > static_cast<int>(model().get_config().powertrain_config->gear_ratio.size()) ||
      gear_num < 1)
    {
      printf("Gear number of %u is not possible.", gear_num);
      return 0.0;
    }
    const auto sample_engine_torque = bilinear_interpolate(
      model().get_config().powertrain_config->torque_v_rpm_throttle, engine_rpm, throttle_level,
      false);
    const auto wheel_torque = sample_engine_torque *
      (model().get_config().powertrain_config->gear_ratio[gear_num - 1] *
      model().get_config().powertrain_config->final_drive_ratio);
    const auto force_wheels = wheel_torque / model().get_config().rear_tyre_config->radius;
    const auto resistance = calc_resistance(vehicle_model_->getVx());
    const auto net_force = force_wheels + resistance;
    return net_force;
  }

  double calc_acceleration(
    const double & throttle_level, const double & brake_level,
    const int & gear_num)
  {
    // Step 1: Throttle and Brake to Engine/Wheel Torque
    double vehicle_speed = vehicle_model_->getVx();
    double engine_rpm = calc_engine_rpm(vehicle_speed, gear_num);
    double engine_force = calc_throttle_force(throttle_level, engine_rpm, gear_num);
    double brake_force = calc_brake_force(brake_level);
    double net_force = engine_force - brake_force;
    double acceleration = net_force / model().get_config().chassis_config->total_mass;
    // Make sure vehicle does not reverse when break is higher than throttle
    if (acceleration < 0 && vehicle_speed <= 0.1) {
      acceleration = 0;
    }
    // make sure it velocitry is zero when no trottle
    if (vehicle_model_->getVx() < 1 && acceleration <= 0) {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(vehicle_model_->getDimX());
      state(0) = vehicle_model_->getX();  // Assuming second element for acceleration
      state(1) = vehicle_model_->getY();  // Assuming first element for steering
      state(2) = vehicle_model_->getYaw();  // Assuming first element for steering
      state(3) = 0;
      state(4) = vehicle_model_->getSteer();
      state(5) = 0;
      vehicle_model_->setState(state);
    }
    return acceleration;
  }

private:
  std::shared_ptr<SimModelInterface> vehicle_model_;
  rclcpp::Subscription<race_msgs::msg::VehicleManualControlCommand>::SharedPtr command_subscriber_;
  // configurations required
  double wheelbase;
  double dt_;
  double steering_angle = 0.0;
  double max_steer;
  double max_steer_rate;
  double max_speed;
  rclcpp::Time last_update_time_;
  race::ttl::TtlIndex target_ttl_index_;
  race::ttl::Position latest_pos_;
  race::ttl::TtlTree::UniquePtr ttl_tree_;
};
}  // namespace race::ghost_car
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(race::ghost_car::ManualController, race::ghost_car::GhostCarController)
