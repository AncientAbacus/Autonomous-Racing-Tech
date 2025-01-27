// Copyright 2023 C.K Wolfe
#include "carla_interface/carla_interface.hpp"

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

namespace race
{
namespace interfaces
{
namespace carla_interface
{
CarlaInterfaceNode::CarlaInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("carla_interface_node", options)
{
  carla_egovehicle_control_publisher_ =
    this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
    "/carla/ego_vehicle/vehicle_control_cmd", 10);

  // subscribe to race_msgs/VehicleControlCommand and publish carla_msgs/CarlaEgoVehicleControl
  race_vehicle_control_subscriber_ =
    this->create_subscription<race_msgs::msg::VehicleControlCommand>(
    "race_control", rclcpp::SensorDataQoS(),
    [this](const race_msgs::msg::VehicleControlCommand::SharedPtr msg) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "race_vehicle_control_subscriber_ >> "
        "Subscribing from race_msgs::msg::VehicleControlCommand ");
      auto carla_msg = carla_msgs::msg::CarlaEgoVehicleControl();
      carla_msg.header.stamp = this->now();
      carla_msg.throttle = msg->accelerator_cmd;
      carla_msg.steer = msg->steering_cmd;
      carla_msg.brake = msg->brake_cmd;
      carla_msg.gear = msg->gear_cmd;
      carla_msg.hand_brake = false;
      carla_msg.reverse = (msg->speed_cmd < 0);
      // TODO(redspry): Map other fields if the corresponding data is available
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "race_vehicle_control_subscriber_ >> "
        "Publishing to carla_msgs::msg::CarlaEgoVehicleControl");
      carla_egovehicle_control_publisher_->publish(carla_msg);
    });

  // subscribe to carla_msgs/CarlaEgoVehicleStatus and publish race_msgs/VehicleKinematicState
  race_vehicle_state_publisher_ =
    this->create_publisher<race_msgs::msg::VehicleKinematicState>(
    "vehicle_state",
    rclcpp::SensorDataQoS());

  carla_egovehicle_status_subscriber_ =
    this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
    "carla/ego_vehicle/vehicle_status", 10,
    [this](const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg) {
      auto race_msg = race_msgs::msg::VehicleKinematicState();
      // Mapping from CarlaEgoVehicleStatus to VehicleKinematicState
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        " carla_egovehicle_status_subscriber_ >> "
        "Subscribing from carla_msgs::msg::CarlaEgoVehicleStatus");
      race_msg.velocity.twist.linear.x = msg->velocity;
      race_msg.accel.accel.linear.x = msg->acceleration.linear.x;
      race_msg.accel.accel.linear.y = msg->acceleration.linear.y;
      race_msg.accel.accel.linear.z = msg->acceleration.linear.z;
      // TODO(redspry): Map other fields if the corresponding data is available
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        " carla_egovehicle_status_subscriber_ >> "
        "Publishing to race_msgs::msg::VehicleKinematicState");
      race_vehicle_state_publisher_->publish(race_msg);
    });

  // Create a timer to publish messages at 100 Hz.
  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration::from_seconds(0.01),
    std::bind(
      &CarlaInterfaceNode::timerCallback,
      this
    )
  );
}

void CarlaInterfaceNode::timerCallback()
{}
}  // namespace carla_interface
}  // namespace interfaces
}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::interfaces::carla_interface::CarlaInterfaceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
