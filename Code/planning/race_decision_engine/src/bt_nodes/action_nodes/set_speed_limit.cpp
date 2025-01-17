// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/set_speed_limit.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "race_msgs/msg/stop_type.hpp"
#include "race_msgs/msg/strategy_type.hpp"
#include "ttl.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace bt
{
namespace action_nodes
{

SetSpeedLimit::SetSpeedLimit(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList SetSpeedLimit::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<std::string>("speed_type"),
    BT::InputPort<double>("current_round_speed"),
    BT::OutputPort<double>("speed_limit"),
  };
}

BT::NodeStatus SetSpeedLimit::tick()
{
  auto speed_type = config().blackboard->get<std::string>("speed_type");
  double speed_limit = 0.0;
  if (speed_type == "stop") {
    speed_limit = 0.0;
  } else if (speed_type == "round_overtaking") {
    if (!config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->use_params)
    {
      speed_limit = config().blackboard->get<double>("current_round_speed") +
        config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->speed_modifiers.overtaking;
    }
  } else if (speed_type == "round_catchup") {
    if (!config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->use_params)
    {
      speed_limit = config().blackboard->get<double>("current_round_speed") +
        config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->speed_modifiers.catchup;
    }
  } else if (speed_type == "pit_lane") {
    speed_limit = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->speed_limit.pit_lane;
  } else if (speed_type == "pit_crawl") {
    speed_limit = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->speed_limit.pit_crawl;
  } else if (speed_type == "pit_road") {
    speed_limit = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->speed_limit.pit_road;
  } else if (speed_type == "yellow_catchup") {
    if (!config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->use_params)
    {
      speed_limit = std::min(
        config().blackboard->get<double>("current_round_speed") +
        config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
          "rde_params")->speed_modifiers.catchup,
        config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
          "rde_params")->speed_limit.yellow);
    } else {
      speed_limit = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->speed_limit.yellow;
    }
  } else {
    if (speed_type == "yellow") {
      speed_limit = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->speed_limit.yellow;
    } else if (speed_type == "green") {
      speed_limit = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->speed_limit.green;
    } else if (speed_type == "round") {
      speed_limit = config().blackboard->get<double>("current_round_speed");
    } else {
      speed_limit = 0.0;
      RCLCPP_ERROR(
        logger_, "Invalid speed type %s", speed_type.c_str());
    }
    if (!config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->use_params)
    {
      auto round_speed = config().blackboard->get<double>("current_round_speed");
      if (speed_limit > round_speed) {
        speed_limit = round_speed;
        RCLCPP_INFO_THROTTLE(
          logger_, clock_,
          config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
            "rde_params")->non_critical_log_time_period_ms,
          "Overriding speed limit %f to round speed %f based of speed type %s",
          config().blackboard->get<double>("speed_limit"), speed_limit,
          speed_type.c_str());
      }
    }
  }
  auto max_speed = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
    "rde_params")->speed_limit.max;
  if (speed_limit > max_speed) {
    speed_limit = max_speed;
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms,
      "Overriding speed limit %f to max speed %f based of speed type %s",
      config().blackboard->get<double>("speed_limit"), speed_limit,
      speed_type.c_str());
  }
  config().blackboard->set<double>("speed_limit", speed_limit);
  RCLCPP_INFO_THROTTLE(
    logger_, clock_,
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms,
    "Setting speed limit %f based on speed type %s",
    config().blackboard->get<double>("speed_limit"), speed_type.c_str());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
