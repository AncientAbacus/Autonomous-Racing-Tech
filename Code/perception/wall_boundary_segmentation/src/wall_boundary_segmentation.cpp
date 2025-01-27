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
 * File Name:   wall_boundary_segmentation.cpp
 * Description: The main 3D LiDAR perception algorithm to segment race wall boundaries for a
 *              high speed autonomous racecar.
 * History:     IAC @ Texas Motor Speedway
 */

#include "wall_boundary_segmentation.hpp"

#include <vector>

namespace race
{

WallBoundarySegmentation::WallBoundarySegmentation(rclcpp::Node * node)
: node_(node),
  input_cloud(),
  inliers_distance_threshold(0.1),
  perpendicular_axis(Eigen::Vector3f(0, 1, 0)),
  epsilon_angle(pcl::deg2rad(45.0)),
  wall_height_threshold(100.0),
  wall_width_threshold(100.0),
  cluster_tolerance(0.02),
  max_cluster_size(250000.0),
  min_cluster_size(100.0),
  fence_to_wall_distance(0.1),
  info_throttle_dt(2000) {}

/**
 * Main race wall segmentation algorithm.
 * @param output_pointcloud Output parameter for the point cloud of the segmented wall boundary.
 * @param output_coefficients Output parameter for the estimated perpendicular plane of the wall.
 * @return Nothing.
 */
void WallBoundarySegmentation::segment(
  pcl::PointCloud<pcl::PointXYZ> & output_pointcloud,
  pcl::ModelCoefficients & output_coefficients)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(inliers_distance_threshold);
  seg.setAxis(perpendicular_axis);
  seg.setEpsAngle(epsilon_angle);
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    output_pointcloud = pcl::PointCloud<pcl::PointXYZ>();
    output_coefficients.values = std::vector<float>(seg.getModel()->getModelSize(), 0);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(), info_throttle_dt,
      "Could not estimate a planar model for the given dataset.");
    return;
  }

  double a = coefficients->values[0];
  double b = coefficients->values[1];
  double c = coefficients->values[2];
  double d = coefficients->values[3];
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(), info_throttle_dt, "Plane segment: %.2fx + %.2fy + %.2fz + %.2f = 0", a, b,
    c, d);

  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int index : inliers->indices) {
    wall_pointcloud->points.push_back(input_cloud->points[index]);
  }
  wall_pointcloud->width = wall_pointcloud->points.size();
  wall_pointcloud->height = 1;
  wall_pointcloud->is_dense = true;

  Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
  Eigen::Vector4f xyz_centroid = Eigen::Vector4f::Zero();
  pcl::compute3DCentroid(*wall_pointcloud, xyz_centroid);
  pcl::computeCovarianceMatrix(*wall_pointcloud, xyz_centroid, covariance_matrix);

  // std::cerr << "wall_pointcloud size: " << wall_pointcloud->points.size() << std::endl;
  // std::cerr << "3D Centroid:\n" << xyz_centroid << std::endl;
  // std::cerr << "Convariance Matrix:\n" << covariance_matrix << std::endl;

  double mu = xyz_centroid[2];
  double std_dev = sqrt(covariance_matrix(2, 2));
  double wall_height = 4.0 * std_dev;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), info_throttle_dt, "mu: %f", mu);
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(), info_throttle_dt, "std_dev: %f", std_dev);
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(), info_throttle_dt, "Wall height: %f", wall_height);

  double wall_width = 4.0 * sqrt(covariance_matrix(1, 1));

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(), info_throttle_dt, "Wall width:: %f", wall_width);
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(), info_throttle_dt, "Wall width threshold: %f", wall_width_threshold);

  if (wall_height < wall_height_threshold || wall_width > wall_width_threshold) {
    output_pointcloud = pcl::PointCloud<pcl::PointXYZ>();
    output_coefficients.values = std::vector<float>(seg.getModel()->getModelSize(), 0);
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(), info_throttle_dt,
      "Detected plane does not have the variance profile of a wall.");
    return;
  }

  // TODO(github/jimenezjose): Remove fence noise in wall segmentation via euclidean clustering.
  output_pointcloud = *wall_pointcloud;
  output_coefficients = *coefficients;
}

}  // namespace race
