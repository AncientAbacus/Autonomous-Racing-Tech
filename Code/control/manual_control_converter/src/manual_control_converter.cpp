// Copyright 2022 Siddharth Saha

#include "manual_control_converter/manual_control_converter.hpp"

#include <memory>

namespace race::manual_control_converter
{

ManualControlConverter::ManualControlConverter(const rclcpp::NodeOptions & options)
: Node("manual_control_converter_node", options)
{
  sub_vehicle_control_command_ =
    this->create_subscription<race_msgs::msg::VehicleManualControlCommand>(
    "manual_cmd",
    rclcpp::SensorDataQoS(),
    std::bind(&ManualControlConverter::callback, this, std::placeholders::_1)
    );
  pub_vehicle_command_ = this->create_publisher<race_msgs::msg::VehicleControlCommand>(
    "cmd", rclcpp::SensorDataQoS());
}

void ManualControlConverter::callback(race_msgs::msg::VehicleManualControlCommand::SharedPtr msg)
{
  pub_vehicle_command_->publish(msg->vehicle_control_command);
}

}  // namespace race::manual_control_converter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::manual_control_converter::ManualControlConverter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
