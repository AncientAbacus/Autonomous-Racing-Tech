// Copyright 2022 Siddharth Saha

#ifndef MANUAL_CONTROL_CONVERTER__MANUAL_CONTROL_CONVERTER_HPP_
#define MANUAL_CONTROL_CONVERTER__MANUAL_CONTROL_CONVERTER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"

namespace race::manual_control_converter
{

class ManualControlConverter : public rclcpp::Node
{
public:
  explicit ManualControlConverter(const rclcpp::NodeOptions & options);

private:
  void callback(race_msgs::msg::VehicleManualControlCommand::SharedPtr msg);

  rclcpp::Publisher<race_msgs::msg::VehicleControlCommand>::SharedPtr pub_vehicle_command_;
  rclcpp::Subscription<race_msgs::msg::VehicleManualControlCommand>::SharedPtr
    sub_vehicle_control_command_;
};

}  // namespace race::manual_control_converter

#endif  // MANUAL_CONTROL_CONVERTER__MANUAL_CONTROL_CONVERTER_HPP_
