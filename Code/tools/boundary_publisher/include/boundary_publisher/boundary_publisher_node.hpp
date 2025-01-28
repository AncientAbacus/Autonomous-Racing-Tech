// Copyright 2023 Siddharth Saha

#ifndef BOUNDARY_PUBLISHER__BOUNDARY_PUBLISHER_NODE_HPP_
#define BOUNDARY_PUBLISHER__BOUNDARY_PUBLISHER_NODE_HPP_

#include <vector>
#include <unordered_map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ttl.hpp"
#include "ttl_tree.hpp"
#include "base_common/pubsub.hpp"

#include "nav_msgs/msg/path.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"

namespace race::tools::boundary_publisher
{

class BoundaryPublisherNode : public rclcpp::Node
{
public:
  explicit BoundaryPublisherNode(const rclcpp::NodeOptions & options);

private:
  void step();

  rclcpp::TimerBase::SharedPtr step_timer_;

  pubsub::MsgSubscriber<race_msgs::msg::TargetTrajectoryCommand>::SharedPtr sub_ttc_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_left_boundary_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_right_boundary_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_actual_path_;

  std::unordered_map<race::ttl::TtlIndex, nav_msgs::msg::Path> left_boundary_map_;
  std::unordered_map<race::ttl::TtlIndex, nav_msgs::msg::Path> right_boundary_map_;
  std::unordered_map<race::ttl::TtlIndex, nav_msgs::msg::Path> actual_path_map_;

  race::ttl::TtlTree::UniquePtr ttl_tree_;
};

}  // namespace race::tools::boundary_publisher

#endif  // BOUNDARY_PUBLISHER__BOUNDARY_PUBLISHER_NODE_HPP_
