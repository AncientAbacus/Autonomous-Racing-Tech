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
 * File Name:   wall_boundary_segmentation.hpp
 * Description: The main 3D LiDAR perception algorithm to segment race wall boundaries for a
 *              high speed autonomous racecar.
 * History:     IAC @ Texas Motor Speedway
 */

#ifndef WALL_BOUNDARY_SEGMENTATION_HPP_
#define WALL_BOUNDARY_SEGMENTATION_HPP_

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
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Geometry>
#include <unordered_map>
#include <limits>
#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "std_msgs/msg/header.hpp"

namespace race
{
/**
  * Race wall boundary segmentation algorithm at high vehicluar speeds over a 3D point cloud.
  *
  * Sources of Help:
  *     - Planar Segmenation: https://pcl.readthedocs.io/en/latest/planar_segmentation.html
  *     - Displaying filtered point clouds: https://gist.github.com/ntraft/c9e74e31cebb25b9be4c
  *     - PCL tutorial: http://www.jeffdelmerico.com/wp-content/uploads/2014/03/pcl_tutorial.pdf
  *     - Voxel Grid: https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html
  **/
class WallBoundarySegmentation
{
public:
  explicit WallBoundarySegmentation(rclcpp::Node * node);

  /**
   * Main race wall segmentation algorithm.
   * @param output_pointcloud Output parameter for the point cloud of the segmented wall boundary.
   * @param output_coefficients Output parameter for the estimated perpendicular plane of the wall.
   * @return Nothing.
   */
  void segment(
    pcl::PointCloud<pcl::PointXYZ> & output_pointcloud,
    pcl::ModelCoefficients & output_coefficients);

  /** Set the input cloud state. */
  inline void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
  {
    this->input_cloud = cloud;
  }

  /** Set the ransac inliers distance threshold. */
  inline void setDistanceThreshold(double distance_threshold)
  {
    this->inliers_distance_threshold = distance_threshold;
  }

  /** Set perpendicular axis of wall in the ego poing of reference. */
  inline void setPerpendicularAxis(Eigen::Vector3f perpendicular_axis)
  {
    this->perpendicular_axis = perpendicular_axis;
  }

  /** Set acceptable perpendicular estimation error in radians. */
  inline void setEpsAngle(double epsilon_angle)
  {
    this->epsilon_angle = epsilon_angle;
  }

  /** Set acceptable tail length of guassian distribution of the inliers that segment the wall. */
  inline void setWallHeightThreshold(double wall_height_threshold)
  {
    this->wall_height_threshold = wall_height_threshold;
  }

  /** Set estimated width of the boundary wall. */
  inline void setWallWidthThreshold(double wall_width_threshold)
  {
    this->wall_width_threshold = wall_width_threshold;
  }

  /** Set output cycles to throttle the info debug statements. */
  inline void setInfoThrottleDt(int info_throttle_dt)
  {
    this->info_throttle_dt = info_throttle_dt;
  }

  /** Set euclidean clustering tolerance. */
  inline void setClusterTolerance(double cluster_tolerance)
  {
    this->cluster_tolerance = cluster_tolerance;
  }

  /** Set max clustering size allowed for euclidean clustering. */
  inline void setMaxClusterSize(double max_cluster_size)
  {
    this->max_cluster_size = max_cluster_size;
  }

  /** Set min clustering size allowed for euclidean clustering. */
  inline void setMinClusterSize(double min_cluster_size)
  {
    this->min_cluster_size = min_cluster_size;
  }

  /** Set the expected distance from the fence to the wall for noise reduction. */
  inline void setFenceToWallDistance(double fence_to_wall_distance)
  {
    this->fence_to_wall_distance = fence_to_wall_distance;
  }

private:
  rclcpp::Node * node_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
  double inliers_distance_threshold;
  Eigen::Vector3f perpendicular_axis;
  double epsilon_angle;
  double wall_height_threshold;
  double wall_width_threshold;
  double cluster_tolerance;
  double max_cluster_size;
  double min_cluster_size;
  double fence_to_wall_distance;
  int info_throttle_dt;
};

}  // namespace race

#endif  // WALL_BOUNDARY_SEGMENTATION_HPP_
