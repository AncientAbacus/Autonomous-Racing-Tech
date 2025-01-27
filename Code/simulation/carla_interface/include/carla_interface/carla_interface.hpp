// Copyright 2023 C.K Wolfe
#ifndef CARLA_INTERFACE__CARLA_INTERFACE_HPP_
#define CARLA_INTERFACE__CARLA_INTERFACE_HPP_

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
class CarlaInterfaceNode : public rclcpp::Node
{
public:
  explicit CarlaInterfaceNode(const rclcpp::NodeOptions & options);

private:
  void timerCallback();

  // Initializations
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr
    carla_egovehicle_control_publisher_;
  rclcpp::Subscription<race_msgs::msg::VehicleControlCommand>::SharedPtr
    race_vehicle_control_subscriber_;
  rclcpp::Publisher<race_msgs::msg::VehicleKinematicState>::SharedPtr race_vehicle_state_publisher_;
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr
    carla_egovehicle_status_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace carla_interface
}  // namespace interfaces
}  // namespace race
#endif  // CARLA_INTERFACE__CARLA_INTERFACE_HPP_
