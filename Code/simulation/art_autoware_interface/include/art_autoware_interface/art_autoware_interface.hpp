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

#ifndef ART_AUTOWARE_INTERFACE__ART_AUTOWARE_INTERFACE_HPP_
#define ART_AUTOWARE_INTERFACE__ART_AUTOWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <race_msgs/msg/steering_report.hpp>
#include <race_msgs/msg/vehicle_kinematic_state.hpp>
#include <race_msgs/msg/vehicle_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <race_msgs/msg/fault_report.hpp>

namespace race
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using ArtSteeringReport = race_msgs::msg::SteeringReport;
using race_msgs::msg::VehicleKinematicState;
using race_msgs::msg::VehicleControlCommand;
using nav_msgs::msg::Odometry;

class ArtAutowareInterfaceNode : public rclcpp::Node
{
public:
  explicit ArtAutowareInterfaceNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr aw_control_pub_{};
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr vks_pub_{};
  rclcpp::Publisher<ArtSteeringReport>::SharedPtr art_steering_report_pub_{};
  rclcpp::Publisher<race_msgs::msg::FaultReport>::SharedPtr fault_report_pub_{};
  rclcpp::Subscription<SteeringReport>::SharedPtr aw_steering_report_sub_{};
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_{};
  rclcpp::Subscription<VehicleControlCommand>::SharedPtr art_control_sub_{};
  rclcpp::TimerBase::SharedPtr vks_pub_timer_{};
  VehicleKinematicState::SharedPtr vks_{};
  Odometry::SharedPtr odom_{};
  ArtSteeringReport::SharedPtr art_steering_report_{};
  AckermannControlCommand::SharedPtr aw_control_cmd_{};
  race_msgs::msg::FaultReport::SharedPtr fault_report_{};

  void on_steering_report(const SteeringReport::SharedPtr msg);
  void on_odom(const Odometry::SharedPtr msg);
  void on_art_control(const VehicleControlCommand::SharedPtr msg);
  void on_vks_pub_timer();
};
}  // namespace race

#endif  // ART_AUTOWARE_INTERFACE__ART_AUTOWARE_INTERFACE_HPP_
