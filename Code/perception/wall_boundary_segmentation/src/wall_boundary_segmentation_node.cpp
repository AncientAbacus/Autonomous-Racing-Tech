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

#include "wall_boundary_segmentation_node.hpp"

#include <limits>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

#include "std_msgs/msg/header.hpp"

namespace race
{

WallBoundarySegmentationNode::WallBoundarySegmentationNode()
: Node("wall_boundary_segmentation_node"),
  inliers_distance_threshold_(0.1),
  voxel_leaf_size_(0.05),
  epsilon_angle_deg_(30.0),
  wall_height_threshold_(100.0),
  wall_width_threshold_(0.0),
  cluster_tolerance_(0.02),  // 2cm
  max_cluster_size_(25000),
  min_cluster_size_(100),
  fence_to_wall_distance_(0.1),
  enable_visualization_(false),
  info_throttle_dt_(2000),
  geo_filter(std::numeric_limits<double>::min(), std::numeric_limits<double>::max())
{
  /* Parameter initialization from param.yaml file. */
  geo_filter.min.x = this->declare_parameter("geometric_filter_min_x", geo_filter.min.x);
  geo_filter.max.x = this->declare_parameter("geometric_filter_max_x", geo_filter.max.x);
  geo_filter.min.y = this->declare_parameter("geometric_filter_min_y", geo_filter.min.y);
  geo_filter.max.y = this->declare_parameter("geometric_filter_max_y", geo_filter.max.y);
  geo_filter.min.z = this->declare_parameter("geometric_filter_min_z", geo_filter.min.z);
  geo_filter.max.z = this->declare_parameter("geometric_filter_max_z", geo_filter.max.z);
  voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", voxel_leaf_size_);
  epsilon_angle_deg_ = this->declare_parameter("epsilon_angle_deg", epsilon_angle_deg_);
  inliers_distance_threshold_ = this->declare_parameter(
    "inliers_distance_threshold",
    inliers_distance_threshold_);
  wall_height_threshold_ = this->declare_parameter("wall_height_threshold", wall_height_threshold_);
  wall_width_threshold_ = this->declare_parameter("wall_width_threshold", wall_width_threshold_);
  enable_visualization_ = this->declare_parameter("enable_visualization", enable_visualization_);
  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", cluster_tolerance_);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", max_cluster_size_);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", min_cluster_size_);
  fence_to_wall_distance_ = this->declare_parameter(
    "fence_to_wall_distance",
    fence_to_wall_distance_);
  info_throttle_dt_ = this->declare_parameter("info_throttle_dt", info_throttle_dt_);
  /* Enable dynamic parameterization via the `ros2 param set` CLI. */
  on_set_parameters_callback_ =
    this->add_on_set_parameters_callback(
    std::bind(
      &WallBoundarySegmentationNode::
      on_set_parameters, this, std::placeholders::_1));
  /* Subscribers */
  pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar_all", rclcpp::SensorDataQoS(), std::bind(
      &WallBoundarySegmentationNode::segmentation_callback, this,
      std::
      placeholders::_1));
  /* Publishers */
  right_wall_segment_publisher_ = this->create_publisher<race_msgs::msg::WallBoundarySegment>(
    "right_wall_segment", rclcpp::SensorDataQoS());
  left_wall_segment_publisher_ = this->create_publisher<race_msgs::msg::WallBoundarySegment>(
    "left_wall_segment", rclcpp::SensorDataQoS());
  right_wall_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "right_wall_points", rclcpp::SensorDataQoS());
  left_wall_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "left_wall_points", rclcpp::SensorDataQoS());
  right_downsampled_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "right_downsampled_points", rclcpp::SensorDataQoS());
  left_downsampled_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "left_downsampled_points", rclcpp::SensorDataQoS());
  right_wall_plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "right_wall_plane", rclcpp::SensorDataQoS());
  left_wall_plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "left_wall_plane", rclcpp::SensorDataQoS());
}

/**
 * Segment the two lateral wall boundaries from the input point cloud.
 * @param cloud Input point cloud with respect to the ego frame of reference.
 * @return Nothing.
 */
void WallBoundarySegmentationNode::segmentation_callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  // Downsample to a Voxel grid.
  // Apply a geometric filter.
  // Remove ground points.
  // In parallel, split the point cloud on its x-axis and segment the two lateral walls:
  //    Compute RANSAC algorithm using a plane estimation model for wall segmentation.
  //    Constrain the plane's normal vector to be close to perpindicular to the y-axis.
  //    Constrain the threshold height of the wall with a guassian distribution model of the
  // inliers on its z-axis.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pointcloud);

  /* Downsample pointcloud to a Voxel Grid. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  double ds = voxel_leaf_size_;
  voxel_grid.setInputCloud(pointcloud);
  voxel_grid.setLeafSize(ds, ds, ds);
  voxel_grid.filter(*voxel_pointcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  applyGeometricFilter(voxel_pointcloud, *downsampled_pointcloud);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), info_throttle_dt_, "Input point cloud: %ld points", pointcloud->size());
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), info_throttle_dt_, "Downsampled point cloud: %ld points",
    downsampled_pointcloud->size());
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), info_throttle_dt_, "Downsample factor: %.2f",
    (1 - static_cast<double>(downsampled_pointcloud->size()) / pointcloud->size()) * 100);

  /* Split the point cloud on its x-axis to segment the two lateral walls. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr right(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr left(new pcl::PointCloud<pcl::PointXYZ>);
  for (pcl::PointXYZ point : *downsampled_pointcloud) {
    if (point.y < 0) {right->push_back(point);} else {left->push_back(point);}
  }

  if (enable_visualization_) {
    sensor_msgs::msg::PointCloud2 right_output;
    pcl::toROSMsg(*right, right_output);
    right_output.header = cloud->header;
    right_downsampled_point_cloud_publisher_->publish(right_output);

    sensor_msgs::msg::PointCloud2 left_output;
    pcl::toROSMsg(*left, left_output);
    left_output.header = cloud->header;
    left_downsampled_point_cloud_publisher_->publish(left_output);
  }

  /* Segment the right wall boundary. */
  race_msgs::msg::WallBoundarySegment right_wall_segment;
  sensor_msgs::msg::PointCloud2 right_wall_points;
  predictWallSegment(right, right_wall_segment, right_wall_points);
  right_wall_segment_publisher_->publish(right_wall_segment);

  if (enable_visualization_) {
    right_wall_points.header = cloud->header;
    right_wall_points_publisher_->publish(right_wall_points);
    sensor_msgs::msg::PointCloud2 right_wall_plane;
    visualizeWallBoundarySegment(right_wall_segment, right_wall_plane);
    right_wall_plane.header = cloud->header;
    right_wall_plane_publisher_->publish(right_wall_plane);
  }

  /* Segment the left wall boundary. */
  race_msgs::msg::WallBoundarySegment left_wall_segment;
  sensor_msgs::msg::PointCloud2 left_wall_points;
  predictWallSegment(left, left_wall_segment, left_wall_points);
  left_wall_segment_publisher_->publish(left_wall_segment);

  if (enable_visualization_) {
    left_wall_points.header = cloud->header;
    left_wall_points_publisher_->publish(left_wall_points);
    sensor_msgs::msg::PointCloud2 left_wall_plane;
    visualizeWallBoundarySegment(left_wall_segment, left_wall_plane);
    left_wall_plane.header = cloud->header;
    left_wall_plane_publisher_->publish(left_wall_plane);
  }
}

/**
 * Predict singular wall segment in pointcloud view using a ransac plane model.
 * @param view Pointcloud in the frame of reference of the ego vehicle with at most one wall present.
 * @param wall_segment Custom race msg that holds the wall inliers as a point cloud and the estimated tangent plane of the wall.
 * @param header Frame of reference and timestamp of the input pointcloud.
 * @return Nothing.
 */
void WallBoundarySegmentationNode::predictWallSegment(
  pcl::PointCloud<pcl::PointXYZ>::Ptr view,
  race_msgs::msg::WallBoundarySegment & wall_segment, sensor_msgs::msg::PointCloud2 & wall_points)
{
  /* Segment the wall boundary. */
  WallBoundarySegmentation predictor(this);
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::ModelCoefficients::Ptr wall_coefficients(new pcl::ModelCoefficients());
  predictor.setInputCloud(view);
  predictor.setDistanceThreshold(inliers_distance_threshold_);
  predictor.setPerpendicularAxis(Eigen::Vector3f(0, 1, 0));
  predictor.setEpsAngle(pcl::deg2rad(epsilon_angle_deg_));
  predictor.setWallHeightThreshold(wall_height_threshold_);
  predictor.setWallWidthThreshold(wall_width_threshold_);
  predictor.setInfoThrottleDt(info_throttle_dt_);
  predictor.setClusterTolerance(cluster_tolerance_);
  predictor.setMaxClusterSize(max_cluster_size_);
  predictor.setMinClusterSize(min_cluster_size_);
  predictor.segment(*wall_pointcloud, *wall_coefficients);

  /* Populate prediction onto output parameter. */
  pcl::toROSMsg(*wall_pointcloud, wall_points);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), info_throttle_dt_, "wall_pointcloud size: %ld",
    wall_pointcloud->points.size());
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), info_throttle_dt_, "wall_coefficients values: %ld",
    wall_coefficients->values.size());

  wall_segment.inliers_size = wall_pointcloud->points.size();
  if (wall_coefficients->values.size() == PLANE_MODEL_SIZE) {
    wall_segment.coefficient_a = wall_coefficients->values[0];
    wall_segment.coefficient_b = wall_coefficients->values[1];
    wall_segment.coefficient_c = wall_coefficients->values[2];
    wall_segment.coefficient_d = wall_coefficients->values[3];
  }
}

/**
 * Filter point cloud by parametrized geometric constraints.
 * @param pointcloud Dataset to get filtered.
 * @param output Dataset after filter.
 * @return Nothing.
 */
void WallBoundarySegmentationNode::applyGeometricFilter(
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::PointCloud<pcl::PointXYZ> & output) const
{
  // Geometric pass through filter
  for (pcl::PointXYZ point : *pointcloud) {
    if (point.x < geo_filter.min.x || point.x > geo_filter.max.x) {
      // x-axis filter
      continue;
    } else if (point.y < geo_filter.min.y || point.y > geo_filter.max.y) {
      // y-axis filter
      continue;
    } else if (point.z < geo_filter.min.z || point.z > geo_filter.max.z) {
      // z-axis filter
      continue;
    }
    output.push_back(point);
  }
}

/**
 * Method to dynamically update parameters via the `ros2 param set` CLI.
 * @param parameters List of parameters to update.
 * @return Success or Failure of setting the parameter to its new value.
 */
rcl_interfaces::msg::SetParametersResult WallBoundarySegmentationNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  static std::unordered_map<std::string, double *> params_type_double_map({
      {"geometric_filter_min_x", &geo_filter.min.x},
      {"geometric_filter_max_x", &geo_filter.max.x},
      {"geometric_filter_min_y", &geo_filter.min.y},
      {"geometric_filter_max_y", &geo_filter.max.y},
      {"geometric_filter_min_z", &geo_filter.min.z},
      {"geometric_filter_max_z", &geo_filter.max.z},
      {"inliers_distance_threshold", &inliers_distance_threshold_},
      {"wall_height_threshold", &wall_height_threshold_},
      {"wall_width_threshold", &wall_width_threshold_},
      {"voxel_leaf_size", &voxel_leaf_size_},
      {"cluster_tolerance", &cluster_tolerance_},
      {"min_cluster_size", &min_cluster_size_},
      {"max_cluster_size", &max_cluster_size_},
      {"fence_to_wall_distance", &fence_to_wall_distance_},
      {"epsilon_angle_deg", &epsilon_angle_deg_}});
  static std::unordered_map<std::string, bool *> params_type_bool_map({
      {"enable_visualization", &enable_visualization_}});
  static std::unordered_map<std::string, int *> params_type_int_map({
      {"info_throttle_dt", &info_throttle_dt_}});
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  // Simply update all params dynamically.
  for (const auto & param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      std::string key = param.get_name();
      if (auto it = params_type_double_map.find(key); it != params_type_double_map.end()) {
        *it->second = param.as_double();
        result.successful = true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      std::string key = param.get_name();
      if (auto it = params_type_bool_map.find(key); it != params_type_bool_map.end()) {
        *it->second = param.as_bool();
        result.successful = true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      std::string key = param.get_name();
      if (auto it = params_type_int_map.find(key); it != params_type_int_map.end()) {
        *it->second = param.as_int();
        result.successful = true;
      }
    }
  }
  return result;
}

/**
  * Visualize the {@link WallBoundarySegment} plane as a {@link PointCloud2} object for rviz.
  * @param wall_segment Normalized cartesian coefficients for the plane estimation wall segment.
  * @param plane_points Pointcloud representation of the estimated cartesian plane.
  * @return Nothing.
  */
void WallBoundarySegmentationNode::visualizeWallBoundarySegment(
  race_msgs::msg::WallBoundarySegment & wall_segment,
  sensor_msgs::msg::PointCloud2 & plane_points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  const int plane_resolution = 200;
  double dx = (geo_filter.max.x - geo_filter.min.x) / plane_resolution;
  double dz = (geo_filter.max.z - geo_filter.min.z) / plane_resolution;
  double a = wall_segment.coefficient_a;
  double b = wall_segment.coefficient_b;
  double c = wall_segment.coefficient_c;
  double d = wall_segment.coefficient_d;

  if (wall_segment.inliers_size == 0) {
    return;
  }

  for (double x = geo_filter.min.x; x < geo_filter.max.x; x += dx) {
    for (double z = geo_filter.min.z; z < geo_filter.max.z; z += dz) {
      double y = (a * x + c * z + d) / (-b);
      pcl::PointXYZ point(x, y, z);
      plane_pointcloud->points.push_back(point);
    }
  }
  pcl::toROSMsg(*plane_pointcloud, plane_points);
}

}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<race::WallBoundarySegmentationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
