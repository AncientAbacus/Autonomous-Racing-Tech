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
 * File Name:   wall_boundary_segmentation_node.cpp
 * Description: 3D LiDAR perception to segment race wall boundaries for a
 *              high speed autonomous racecar.
 * History:     IAC @ Texas Motor Speedway
 */

#ifndef WALL_BOUNDARY_SEGMENTATION_NODE_HPP_
#define WALL_BOUNDARY_SEGMENTATION_NODE_HPP_

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
#include <unordered_map>
#include <limits>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "race_msgs/msg/wall_boundary_segment.hpp"

#include "wall_boundary_segmentation_node.hpp"
#include "wall_boundary_segmentation.hpp"

namespace race
{
/**
 * Segment wall boundaries at high speeds over a 3D point cloud.
 *
 * @author github/jimenezjose (Jose Jimenez-Olivas)
 */
class WallBoundarySegmentationNode : public rclcpp::Node
{
public:
  const uint64_t PLANE_MODEL_SIZE = 4;

  WallBoundarySegmentationNode();

private:
  /* Subscribers */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  /* Publishers */
  rclcpp::Publisher<race_msgs::msg::WallBoundarySegment>::SharedPtr right_wall_segment_publisher_;
  rclcpp::Publisher<race_msgs::msg::WallBoundarySegment>::SharedPtr left_wall_segment_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_wall_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_wall_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_wall_plane_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_wall_plane_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    right_downsampled_point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    left_downsampled_point_cloud_publisher_;

  /* Node parameters. */
  double inliers_distance_threshold_;
  double voxel_leaf_size_;
  double epsilon_angle_deg_;
  double wall_height_threshold_;
  double wall_width_threshold_;
  double cluster_tolerance_;
  double max_cluster_size_;
  double min_cluster_size_;
  double fence_to_wall_distance_;
  bool enable_visualization_;
  int info_throttle_dt_;
  /* Data struct to hold multiple params that describe 3D geometric filter constraints. */
  struct GeometricFilter
  {
    struct Min
    {
      double x, y, z;
      explicit Min(double limit)
      : x(limit), y(limit), z(limit) {}
    } min;
    struct Max
    {
      double x, y, z;
      explicit Max(double limit)
      : x(limit), y(limit), z(limit) {}
    } max;
    GeometricFilter(double min_threshold, double max_threshold)
    : min{min_threshold}, max{max_threshold} {}
  } geo_filter{0, 0};
  /* Callback to handle dynamic parameterization. */
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_;

  /**
   * Segment the two lateral wall boundaries from the input point cloud.
   * @param cloud Input point cloud with respect to the ego frame of reference.
   * @return Nothing.
   */
  void segmentation_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  /**
   * Predict singular wall segment in pointcloud view using a ransac plane model.
   * @param view Pointcloud in the frame of reference of the ego vehicle with at most one wall present.
   * @param wall_segment Custom race msg that holds the wall inliers as a point cloud and the estimated tangent plane of the wall.
   * @param header Frame of reference and timestamp of the input pointcloud.
   * @return Nothing.
   */
  void predictWallSegment(
    pcl::PointCloud<pcl::PointXYZ>::Ptr view,
    race_msgs::msg::WallBoundarySegment & wall_segment,
    sensor_msgs::msg::PointCloud2 & wall_points);

  /**
   * Filter point cloud by parametrized geometric constraints.
   * @param pointcloud Dataset to get filtered.
   * @param output Dataset after filter.
   * @return Nothing.
   */
  void applyGeometricFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,
    pcl::PointCloud<pcl::PointXYZ> & output) const;

  /**
   * Method to dynamically update parameters via the `ros2 param set` CLI.
   * @param parameters List of parameters to update.
   * @return Success or Failure of setting the parameter to its new value.
   */
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * Visualize the {@link WallBoundarySegment} plane as a {@link PointCloud2} object for rviz.
   * @param wall_segment Normalized cartesian coefficients for the plane estimation wall segment.
   * @param plane_points Pointcloud representation of the estimated cartesian plane.
   * @return Nothing.
   */
  void visualizeWallBoundarySegment(
    race_msgs::msg::WallBoundarySegment & wall_segment,
    sensor_msgs::msg::PointCloud2 & plane_points);
};

}  // namespace race

#endif  // WALL_BOUNDARY_SEGMENTATION_NODE_HPP_
