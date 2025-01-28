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

#include <algorithm>
#include <numeric>
#include <vector>
#include <memory>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robot_localization_interface/robot_localization_interface_node.hpp"

namespace race
{
namespace localization
{
RobotLocalizationInterfaceNode::RobotLocalizationInterfaceNode(const rclcpp::NodeOptions & options)
: Node("robot_localization_interface_node", options),
  sync_(sub_odom_, sub_accel_, 5),
  tf_helper_(*this)
{
  cg_to_front_ = declare_parameter<double>("cg_to_front");
  min_speed_thresh_yaw_ = declare_parameter<double>("min_speed_thresh_yaw");
  declare_parameter<bool>("publish_tf", false);
  pub_slip_ = create_publisher<std_msgs::msg::Float64>("slip", rclcpp::QoS{5});
  pub_vks_ = create_publisher<race_msgs::msg::VehicleKinematicState>("state", rclcpp::QoS{5});
  sub_odom_.subscribe(this, "odom");
  sub_accel_.subscribe(this, "accel");
  sync_.registerCallback(
    std::bind(
      &RobotLocalizationInterfaceNode::on_localization_output, this,
      std::placeholders::_1, std::placeholders::_2));
  sub_steer_ = create_subscription<race_msgs::msg::SteeringReport>(
    "steering_report",
    rclcpp::SensorDataQoS(), [this](race_msgs::msg::SteeringReport::ConstSharedPtr msg) {
      steer_angle_ = msg;
    });
}

void RobotLocalizationInterfaceNode::on_localization_output(
  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr accel_msg)
{
  if (!get_parameter("publish_tf").as_bool()) {
    return;
  }

  race_msgs::msg::VehicleKinematicState vks;
  if (steer_angle_) {
    vks.front_wheel_angle_rad = steer_angle_->front_wheel_angle_rad;
  } else {
    vks.source_status_code |= 1 << race_msgs::msg::VehicleKinematicState::SOURCE_WHEEL_ANGLE_BAD;
  }
  vks.header = odom_msg->header;
  vks.accel = accel_msg->accel;
  vks.velocity = odom_msg->twist;
  vks.pose = odom_msg->pose;
  vks.car_yaw = TransformHelper::heading_from_quaternion(odom_msg->pose.pose.orientation);
  vks.car_yaw_rate = odom_msg->twist.twist.angular.z;
  vks.velocity_yaw = vks.car_yaw;
  if (vks.speed_mps >= min_speed_thresh_yaw_) {
    vks.velocity_yaw += atan2(odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.x);
  }
  vks.child_frame_id = "cg";
  vks.speed_mps = std::hypot(
    odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y,
    odom_msg->twist.twist.linear.z);
  pub_vks_->publish(vks);

  std_msgs::msg::Float64 slip;
  const auto cg_slip = (atan2(odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.x));
  const auto front_slip = -cg_slip + cg_to_front_ * odom_msg->twist.twist.angular.z /
    odom_msg->twist.twist.linear.x * 180.0 / M_PI;
  slip.data = front_slip;
  pub_slip_->publish(slip);

  if (get_parameter("publish_tf").as_bool()) {
    geometry_msgs::msg::TransformStamped cg_to_base_link, map_to_base_link;
    geometry_msgs::msg::PoseWithCovariance base_link_pose;
    tf2::Transform tf2_map_to_cg, tf2_cg_to_base_link, tf2_map_to_base_link;
    tf_helper_.lookup_transform(
      "base_link", odom_msg->child_frame_id, cg_to_base_link,
      odom_msg->header.stamp);
    tf2::fromMsg(cg_to_base_link.transform, tf2_cg_to_base_link);
    tf2::fromMsg(odom_msg->pose.pose, tf2_map_to_cg);
    tf2_map_to_base_link = tf2_map_to_cg * tf2_cg_to_base_link;
    map_to_base_link.transform = tf2::toMsg(tf2_map_to_base_link);
    map_to_base_link.header = odom_msg->header;
    map_to_base_link.child_frame_id = "base_link";
    tf_helper_.send_transform(map_to_base_link);
  }
}
}  // namespace localization
}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::localization::RobotLocalizationInterfaceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
