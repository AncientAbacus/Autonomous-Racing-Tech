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
 * File Name:   race_boundary_localizer_node.hpp
 * Description: Reaactive LiDAR localization using race boundary wall segmentation for
 *              high-speed autonomous racecars at the Indy Autonomouse Challenge.
 * History:     IAC @ Texas Motor Speedway
 */

#ifndef RACE_BOUNDARY_LOCALIZER_NODE_HPP_
#define RACE_BOUNDARY_LOCALIZER_NODE_HPP_

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/angles.h>
#include <Eigen/Geometry>

#include <chrono>
#include <functional>
#include <memory>
#include <unordered_map>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "race_msgs/msg/wall_boundary_segment.hpp"


namespace race
{
/**
 * Reactive LiDAR localization to estimate the ego vehicular pose at high speeds.
 *
 * Sources of Help:
 *     - https://mitpress.mit.edu/9780262201629/probabilistic-robotics/
 *     - https://www.researchgate.net/publication/357884857_A_Combined_LiDAR-Camera_Localization_for_Autonomous_Race_Cars
 *     - https://arxiv.org/abs/1910.01122
 *
 * @author github/jimenezjose (Jose Jimenez-Olivas)
 */
class RaceBoundaryLocalizerNode : public rclcpp::Node
{
public:
  RaceBoundaryLocalizerNode();

private:
  rclcpp::Subscription<race_msgs::msg::WallBoundarySegment>::SharedPtr
    right_wall_points_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_to_wall_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_from_wall_publisher_;

  /* Node parameters. */
  int freshness_threshold_;
  int info_throttle_dt_;
  /* Callback to handle dynamic parameterization. */
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_;

  /**
   * Segment race wall boundaries, using a perpendicular plane model estimation model, then estimate vehicular pose in a Franet frame.
   * Notice that only the lateral displacement and heading is estimated from this LiDAR wall segmentation localizer.
   * To achieve longitudinal position, a monocular camera using visual SLAM wil run in parallel to complete the
   * Frenet frame estimation of the ego vehicle.
   * @param wall_segment Plane model estimation of segmented race wall boundary.
   * @return Nothing.
   */
  void wall_boundary_callback(race_msgs::msg::WallBoundarySegment::SharedPtr wall_segment);

  /**
  * Method to dynamically update parameters via the `ros2 param set` CLI.
  * @param parameters List of parameters to update.
  * @return Success or Failure of setting the parameter to its new value.
  */
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace race

#endif  // RACE_BOUNDARY_LOCALIZER_NODE_HPP_
