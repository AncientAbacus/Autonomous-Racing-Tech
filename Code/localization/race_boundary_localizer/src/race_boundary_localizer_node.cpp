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
/**
 *                            AI Racing Tech
 *                 University of California, San Diego
 *
 * File Name:   race_boundary_localizer_node.cpp
 * Description: Reactive LiDAR localization using race boundary wall segmentation for
 *              high-speed autonomous racecars at the Indy Autonomouse Challenge.
 * History:     IAC @ Texas Motor Speedway
 */

#include "race_boundary_localizer_node.hpp"

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

namespace race
{

RaceBoundaryLocalizerNode::RaceBoundaryLocalizerNode()
: Node("race_boundary_localizer_node"),
  freshness_threshold_(200),
  info_throttle_dt_(2000)
{
  /* Parameter initialization from param.yaml file. */
  freshness_threshold_ = this->declare_parameter("freshness_threshold", freshness_threshold_);
  info_throttle_dt_ = this->declare_parameter("info_throttle_dt", info_throttle_dt_);
  /* Enable dynamic parameterization via the `ros2 param set` CLI. */
  on_set_parameters_callback_ =
    this->add_on_set_parameters_callback(
    std::bind(
      &RaceBoundaryLocalizerNode::on_set_parameters,
      this, std::placeholders::_1));

  right_wall_points_subscriber_ = this->create_subscription<race_msgs::msg::WallBoundarySegment>(
    "/perception/right_wall_segment", rclcpp::SensorDataQoS(),
    std::bind(&RaceBoundaryLocalizerNode::wall_boundary_callback, this, std::placeholders::_1));
  distance_to_wall_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "distance_to_wall",
    rclcpp::SensorDataQoS());
  heading_from_wall_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "heading_from_wall",
    rclcpp::SensorDataQoS());
}

/**
  * Segment race wall boundaries, using a perpendicular plane model estimation model, then estimate vehicular pose in a Frenet frame.
  * Notice that only the lateral displacement and heading is estimated from this LiDAR wall segmentation localizer.
  * To achieve longitudinal position, a monocular camera using visual SLAM wil run in parallel to complete the
  * Frenet frame estimation of the ego vehicle.
  * @param wall_segment Plane model estimation of tangential segmented race wall boundary.
  * @return Nothing.
  */
void RaceBoundaryLocalizerNode::wall_boundary_callback(
  race_msgs::msg::WallBoundarySegment::SharedPtr wall_segment)
{
  if (wall_segment->inliers_size == 0) {
    return;
  }

  /* Normalized cartesian coefficients of estimated RANSAC plane model. */
  double a = wall_segment->coefficient_a;
  double b = wall_segment->coefficient_b;
  double c = wall_segment->coefficient_c;
  double d = wall_segment->coefficient_d;

  /* Shortest distance to plane. */
  pcl::PointXYZ origin(0, 0, 0);
  Eigen::Vector4f plane_coefficients(a, b, c, d);
  double distance_to_wall = pcl::pointToPlaneDistance(origin, plane_coefficients);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), info_throttle_dt_, "Shortest distance to wall: %f", distance_to_wall);

  std_msgs::msg::Float64 distance_to_wall_message;
  distance_to_wall_message.data = distance_to_wall;
  distance_to_wall_publisher_->publish(distance_to_wall_message);

  /* Relative yaw of ego vehicle. */
  Eigen::Vector3f tangent_normal(a, b, 0);
  Eigen::Vector3f ego_orthogonal(0, -1, 0);
  double yaw = pcl::getAngle3D(tangent_normal, ego_orthogonal);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), info_throttle_dt_, "Relative yaw of ego vehicle: %f", yaw);

  std_msgs::msg::Float64 heading_from_wall_message;
  heading_from_wall_message.data = pcl::rad2deg(yaw);
  heading_from_wall_publisher_->publish(heading_from_wall_message);
}

/**
  * Method to dynamically update parameters via the `ros2 param set` CLI.
  * @param parameters List of parameters to update.
  * @return Success or Failure of setting the parameter to its new value.
  */
rcl_interfaces::msg::SetParametersResult RaceBoundaryLocalizerNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  static std::unordered_map<std::string, int *> params_type_int_map({
      {"freshness_threshold", &freshness_threshold_},
      {"info_throttle_dt", &info_throttle_dt_}});
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  // Simply update all params dynamically.
  for (const auto & param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      std::string key = param.get_name();
      if (auto it = params_type_int_map.find(key); it != params_type_int_map.end()) {
        *it->second = param.as_int();
        result.successful = true;
      }
    }
  }
  return result;
}

}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<race::RaceBoundaryLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
