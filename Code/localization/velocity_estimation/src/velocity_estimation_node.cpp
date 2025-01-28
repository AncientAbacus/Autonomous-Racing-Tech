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

#include "velocity_estimation/velocity_estimation_node.hpp"

namespace race
{
namespace localization
{
VelocityEstimationNode::VelocityEstimationNode(const rclcpp::NodeOptions & options)
: Node("velocity_estimation_node", options)
{
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist",
    rclcpp::QoS{10});
  sub_wheel_ = create_subscription<race_msgs::msg::WheelSpeedReport>(
    "wheel_speed_report",
    rclcpp::SensorDataQoS(), [this](race_msgs::msg::WheelSpeedReport::ConstSharedPtr msg) {
      wheel_speed_ = msg;
    });
  sub_steer_ = create_subscription<race_msgs::msg::SteeringReport>(
    "steering_report",
    rclcpp::SensorDataQoS(), [this](race_msgs::msg::SteeringReport::ConstSharedPtr msg) {
      steer_angle_ = msg;
    });

  model_ = std::make_shared<race::vehicle_model::VehicleModel>(
    race::vehicle_model::VehicleModelConfig::SharedPtr());
  race::vehicle_model::load_parameters(this, *model_);

  step_timer_ =
    rclcpp::create_timer(
    this, this->get_clock(), std::chrono::duration<float>(0.01), [this] {
      twist_publish_timer_callback();
    });
}

void localization::VelocityEstimationNode::twist_publish_timer_callback()
{
  static constexpr auto KMPH2MPS = 1 / 3.6;
  static constexpr auto VX_VX = 0;
  static constexpr auto VY_VY = 7;
  static constexpr auto VZ_VZ = 14;
  static constexpr auto VAX_VAX = 21;
  static constexpr auto VAY_VAY = 28;
  static constexpr auto VAZ_VAZ = 35;

  auto variance = [](const std::vector<float> & data)
    {
      float sum = std::reduce(data.begin(), data.end());
      float mean = sum / data.size();
      float accum = 0.0;
      std::for_each(
        data.begin(), data.end(), [&](const float & d) {
          accum += (d - mean) * (d - mean);
        });
      return accum / (data.size() - 1);
    };

  if (wheel_speed_ && steer_angle_) {
    auto wheel_speeds =
      std::vector<float>{wheel_speed_->front_left, wheel_speed_->front_right,
      wheel_speed_->rear_left, wheel_speed_->rear_right};
    auto var_vx = variance(wheel_speeds);
    // wheel_speeds.erase(std::min_element(wheel_speeds.begin(), wheel_speeds.end()));
    // wheel_speeds.erase(std::max_element(wheel_speeds.begin(), wheel_speeds.end()));
    auto ave_wheel_speed =
      std::reduce(wheel_speeds.begin(), wheel_speeds.end()) / wheel_speeds.size() * KMPH2MPS;

    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    twist.header.frame_id = "base_link";
    twist.header.stamp = now();
    twist.twist.twist.linear.x = ave_wheel_speed;
    const auto vx_to_vax = tan(
      steer_angle_->front_wheel_angle_rad - model_->get_config().steer_config->turn_left_bias) /
      model_->get_config().chassis_config->wheel_base;
    twist.twist.twist.angular.z = ave_wheel_speed * vx_to_vax;
    twist.twist.covariance[VX_VX] = var_vx;
    twist.twist.covariance[VY_VY] = var_vx;
    twist.twist.covariance[VZ_VZ] = 1.0;
    twist.twist.covariance[VAX_VAX] = 0.5;
    twist.twist.covariance[VAY_VAY] = 0.5;
    twist.twist.covariance[VAZ_VAZ] = var_vx * vx_to_vax * vx_to_vax;

    pub_twist_->publish(twist);
  }
}
}  // namespace localization
}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::localization::VelocityEstimationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
