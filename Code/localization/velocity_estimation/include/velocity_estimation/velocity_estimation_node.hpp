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

#ifndef VELOCITY_ESTIMATION__VELOCITY_ESTIMATION_NODE_HPP_
#define VELOCITY_ESTIMATION__VELOCITY_ESTIMATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "race_msgs/msg/wheel_speed_report.hpp"
#include "race_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "vehicle_model/vehicle_model.hpp"
#include "vehicle_model/ros_param_loader.hpp"

namespace race
{
namespace localization
{
class VelocityEstimationNode : public rclcpp::Node
{
public:
  explicit VelocityEstimationNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Subscription<race_msgs::msg::WheelSpeedReport>::SharedPtr sub_wheel_;
  rclcpp::Subscription<race_msgs::msg::SteeringReport>::SharedPtr sub_steer_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  race_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_;
  race_msgs::msg::SteeringReport::ConstSharedPtr steer_angle_;
  race::vehicle_model::VehicleModel::SharedPtr model_;

  void twist_publish_timer_callback();
};
}  // namespace localization
}  // namespace race

#endif  // VELOCITY_ESTIMATION__VELOCITY_ESTIMATION_NODE_HPP_
