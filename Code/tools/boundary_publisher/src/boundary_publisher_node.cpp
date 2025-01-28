// Copyright 2022 Siddharth Saha

#include "boundary_publisher/boundary_publisher_node.hpp"

#include <string>
#include <vector>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "transform_helper/transform_helper.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace race::tools::boundary_publisher
{

BoundaryPublisherNode::BoundaryPublisherNode(const rclcpp::NodeOptions & options)
: Node("boundary_publisher_node", options)
{
  pubsub::subscribe_from(this, sub_ttc_, "target_trajectory_command", rclcpp::SensorDataQoS());
  pubsub::publish_to(this, pub_left_boundary_, "left_boundary", rclcpp::SensorDataQoS());
  pubsub::publish_to(this, pub_right_boundary_, "right_boundary", rclcpp::SensorDataQoS());
  pubsub::publish_to(this, pub_actual_path_, "actual_path", rclcpp::SensorDataQoS());

  auto ttl_dir = declare_parameter<std::string>("ttl_dir");
  auto dt = declare_parameter<double>("dt");
  ttl_tree_ = std::make_unique<race::ttl::TtlTree>(ttl_dir.c_str());
  auto valid_idx = ttl_tree_->get_valid_ttl_index_set();
  nav_msgs::msg::Path left_boundary;
  nav_msgs::msg::Path right_boundary;
  nav_msgs::msg::Path actual_path;
  geometry_msgs::msg::PoseStamped left_pose;
  geometry_msgs::msg::PoseStamped actual_pose;
  geometry_msgs::msg::PoseStamped right_pose;
  std::string frame_id = declare_parameter<std::string>("frame_id");
  left_boundary.header.frame_id = frame_id;
  right_boundary.header.frame_id = frame_id;
  actual_path.header.frame_id = frame_id;
  for (auto index : valid_idx) {
    auto ttl_idx = static_cast<race::ttl::TtlIndex>(index);
    left_boundary.poses.clear();
    right_boundary.poses.clear();
    actual_path.poses.clear();
    left_boundary.poses.reserve(ttl_tree_->get_ttl(ttl_idx).header.loop);
    right_boundary.poses.reserve(ttl_tree_->get_ttl(ttl_idx).header.loop);
    actual_path.poses.reserve(ttl_tree_->get_ttl(ttl_idx).header.loop);
    for (size_t i = 0; i < ttl_tree_->get_ttl(ttl_idx).header.loop; ++i) {
      left_pose.header.frame_id = frame_id;
      left_pose.pose.position.x = ttl_tree_->get_ttl(ttl_idx).waypoints.at(i).left_bound.x;
      left_pose.pose.position.y = ttl_tree_->get_ttl(ttl_idx).waypoints.at(i).left_bound.y;
      left_boundary.poses.push_back(left_pose);
      right_pose.header.frame_id = frame_id;
      right_pose.pose.position.x = ttl_tree_->get_ttl(ttl_idx).waypoints.at(i).right_bound.x;
      right_pose.pose.position.y = ttl_tree_->get_ttl(ttl_idx).waypoints.at(i).right_bound.y;
      right_boundary.poses.push_back(right_pose);
      actual_pose.header.frame_id = frame_id;
      actual_pose.pose.position.x = ttl_tree_->get_ttl(ttl_idx).waypoints.at(i).location.x;
      actual_pose.pose.position.y = ttl_tree_->get_ttl(ttl_idx).waypoints.at(i).location.y;
      actual_path.poses.push_back(actual_pose);
    }
    left_boundary_map_.emplace(ttl_idx, left_boundary);
    right_boundary_map_.emplace(ttl_idx, right_boundary);
    actual_path_map_.emplace(ttl_idx, actual_path);
  }

  step_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(dt), [this] {
      step();
    });
}

void BoundaryPublisherNode::step()
{
  if (!sub_ttc_->has_seen_msg()) {
    return;
  }
  auto current_time = get_clock()->now();
  auto ttl_idx = static_cast<race::ttl::TtlIndex>(sub_ttc_->last_received_msg()->current_ttl_index);
  left_boundary_map_.at(ttl_idx).header.stamp = current_time;
  right_boundary_map_.at(ttl_idx).header.stamp = current_time;
  actual_path_map_.at(ttl_idx).header.stamp = current_time;
  pub_left_boundary_->publish(left_boundary_map_.at(ttl_idx));
  pub_right_boundary_->publish(right_boundary_map_.at(ttl_idx));
  pub_actual_path_->publish(actual_path_map_.at(ttl_idx));
}

}  // namespace race::tools::boundary_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(race::tools::boundary_publisher::BoundaryPublisherNode)
