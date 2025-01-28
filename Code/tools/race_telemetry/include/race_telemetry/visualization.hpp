// Copyright 2024 AI Racing Tech

#ifndef RACE_TELEMETRY__VISUALIZATION_HPP_
#define RACE_TELEMETRY__VISUALIZATION_HPP_

#include <string>
#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "race_msgs/msg/vehicle_telemetry.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/ride_height_report.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/path.hpp"

#include "base_common/pubsub.hpp"
#include "ttl_tree.hpp"

#include "transform_helper/transform_helper.hpp"
#include "race_telemetry/visualizer.hpp"

using pubsub::MsgSubscriber;

namespace race
{
class Visualization : public rclcpp::Node
{
public:
  explicit Visualization(const rclcpp::NodeOptions & options);

private:
  void step(race_msgs::msg::VehicleTelemetry::SharedPtr msg);

  rclcpp::Subscription<race_msgs::msg::VehicleTelemetry>::SharedPtr sub_vehicle_telemetry_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_vis_;
  rclcpp::Time prev_time;
  race_msgs::msg::RdeTelemetry rde_msg_;
  sensor_msgs::msg::Image image_msg_;
  cv_bridge::CvImagePtr cv_ptr_;
  rclcpp::Time start_time;

  double dt_{};
  rclcpp::TimerBase::SharedPtr step_timer_;

  uint8_t vehicle_number_ {};
  uint8_t version_number_ {1};
  uint8_t next_sequence_number_ {};

  TelemetryVisualizer vis_;
  std_msgs::msg::Header vis_header_;
};

}  // namespace race

#endif  // RACE_TELEMETRY__VISUALIZATION_HPP_
