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

#ifndef ROBOT_LOCALIZATION_INTERFACE__ROBOT_LOCALIZATION_INTERFACE_NODE_HPP_
#define ROBOT_LOCALIZATION_INTERFACE__ROBOT_LOCALIZATION_INTERFACE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "race_msgs/msg/steering_report.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "transform_helper/transform_helper.hpp"

namespace race
{
namespace localization
{
class RobotLocalizationInterfaceNode : public rclcpp::Node
{
public:
  explicit RobotLocalizationInterfaceNode(const rclcpp::NodeOptions & options);

private:
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
  message_filters::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped> sub_accel_;
  message_filters::TimeSynchronizer<nav_msgs::msg::Odometry,
    geometry_msgs::msg::AccelWithCovarianceStamped> sync_;

  rclcpp::Subscription<race_msgs::msg::SteeringReport>::SharedPtr sub_steer_;
  rclcpp::Publisher<race_msgs::msg::VehicleKinematicState>::SharedPtr pub_vks_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_slip_;

  race_msgs::msg::SteeringReport::ConstSharedPtr steer_angle_;
  TransformHelper tf_helper_;

  double cg_to_front_;
  double min_speed_thresh_yaw_;

  void on_localization_output(
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr accel_msg);
};
}  // namespace localization
}  // namespace race

#endif  // ROBOT_LOCALIZATION_INTERFACE__ROBOT_LOCALIZATION_INTERFACE_NODE_HPP_
