// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PIT_ENTRANCE_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PIT_ENTRANCE_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "ttl_tree.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace bt
{
namespace condition_nodes
{
class IfInPitEntrance : public BT::ConditionNode
{
public:
  IfInPitEntrance(
    const std::string & name, const BT::NodeConfiguration & config,
    const rclcpp::Logger & logger, rclcpp::Clock & clock,
    const race::ttl::TtlTree::ConstSharedPtr ttl_tree);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree_;
};
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PIT_ENTRANCE_HPP_
