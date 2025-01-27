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

#include <chrono>
#include <memory>
#include "art_autoware_interface/art_autoware_interface.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace race
{
ArtAutowareInterfaceNode::ArtAutowareInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("art_autoware_interface_node", options)
{
  declare_parameter<uint8_t>("fault_report_level", 0);
  declare_parameter<bool>("publish_fault_report", true);
  aw_control_cmd_ = std::make_shared<AckermannControlCommand>();
  aw_control_pub_ = create_publisher<AckermannControlCommand>(
    "autoware/ackermann_control_command",
    1);
  if (declare_parameter<bool>("publish_vks", true)) {
    vks_ = std::make_shared<VehicleKinematicState>();
    vks_pub_ = create_publisher<VehicleKinematicState>("/vehicle/state", rclcpp::SensorDataQoS());
    odom_sub_ =
      create_subscription<Odometry>(
      "/odom", 1,
      [this](const Odometry::SharedPtr msg) {on_odom(msg);});
    vks_pub_timer_ =
      rclcpp::create_timer(
      this, get_clock(),
      std::chrono::duration<float>(declare_parameter<double>("publish_vks_interval_ms", 0.01)),
      [this]() {
        on_vks_pub_timer();
      });
  }
  fault_report_ = std::make_shared<race_msgs::msg::FaultReport>();
  fault_report_pub_ = create_publisher<race_msgs::msg::FaultReport>(
    "/vehicle/low_level_fault_report",
    rclcpp::SensorDataQoS());
  art_steering_report_ = std::make_shared<ArtSteeringReport>();
  art_steering_report_pub_ = create_publisher<ArtSteeringReport>(
    "art/steering_report",
    rclcpp::SensorDataQoS());
  aw_steering_report_sub_ = create_subscription<SteeringReport>(
    "autoware/steering_report", 1, [this](
      const SteeringReport::SharedPtr msg) {on_steering_report(msg);});
  art_control_sub_ =
    create_subscription<VehicleControlCommand>(
    "art/control_command", 1,
    [this](const VehicleControlCommand::SharedPtr msg) {on_art_control(msg);});
}

void ArtAutowareInterfaceNode::on_steering_report(const SteeringReport::SharedPtr msg)
{
  art_steering_report_->stamp = now();
  art_steering_report_->front_wheel_angle_rad = msg->steering_tire_angle;
  art_steering_report_pub_->publish(*art_steering_report_);
  if (vks_) {
    vks_->front_wheel_angle_rad = msg->steering_tire_angle;
  }
}

void ArtAutowareInterfaceNode::on_odom(const Odometry::SharedPtr msg)
{
  auto q = tf2::Quaternion();
  tf2::fromMsg(msg->pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  vks_->car_yaw = static_cast<float>(yaw);

  // Velocity yaw
  if (odom_) {
    vks_->velocity_yaw = atan2(
      msg->pose.pose.position.y - odom_->pose.pose.position.y,
      msg->pose.pose.position.x - odom_->pose.pose.position.x);
  }
  vks_->velocity.twist = msg->twist.twist;

  tf2::Vector3 v{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
  vks_->speed_mps = v.length();

  vks_->pose.pose = msg->pose.pose;

  odom_ = msg;
}

void ArtAutowareInterfaceNode::on_art_control(const VehicleControlCommand::SharedPtr msg)
{
  aw_control_cmd_->stamp = msg->stamp;
  aw_control_cmd_->lateral.stamp = msg->stamp;
  aw_control_cmd_->lateral.steering_tire_angle = msg->steering_cmd;
  aw_control_cmd_->longitudinal.stamp = msg->stamp;
  if (msg->lon_control_type == VehicleControlCommand::LON_CONTROL_THROTTLE) {
    if (msg->brake_cmd == 0.0) {
      aw_control_cmd_->longitudinal.acceleration = msg->accelerator_cmd;
    } else {
      aw_control_cmd_->longitudinal.acceleration = -1.0 * msg->brake_cmd;
    }
  } else if (msg->lon_control_type == VehicleControlCommand::LON_CONTROL_SPEED) {
    aw_control_cmd_->longitudinal.speed = msg->speed_cmd;
  }
  aw_control_pub_->publish(*aw_control_cmd_);
}

void ArtAutowareInterfaceNode::on_vks_pub_timer()
{
  if (odom_) {
    vks_->header.stamp = now();
    vks_->header.frame_id = "map";
    vks_->child_frame_id = "base_link";
    if (get_parameter("publish_vks").as_bool()) {
      vks_pub_->publish(*vks_);
    }
  }
  fault_report_->stop_type.stop_type = get_parameter("fault_report_level").as_int();
  fault_report_->stamp = now();
  if (this->get_parameter("publish_fault_report").as_bool()) {
    fault_report_pub_->publish(*fault_report_);
  }
}
}  // namespace race

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(race::ArtAutowareInterfaceNode)
