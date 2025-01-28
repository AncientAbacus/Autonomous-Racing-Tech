// Copyright 2024 AI Racing Tech

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "race_telemetry/visualization.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "opencv2/opencv.hpp"

using std::chrono::duration;
using std::placeholders::_1;

namespace race
{

Visualization::Visualization(const rclcpp::NodeOptions & options)
: rclcpp::Node("visualization", options), vis_(this, declare_parameter<std::string>(
      "ttl_directory").c_str())
{
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos_profile.best_effort();
  vis_header_.frame_id = "base_link";
  sub_vehicle_telemetry_ = this->create_subscription<race_msgs::msg::VehicleTelemetry>(
    "telemetry", qos_profile, std::bind(&Visualization::step, this, _1));
  pubsub::publish_to(this, pub_vis_, "visualization");
}

void Visualization::step(race_msgs::msg::VehicleTelemetry::SharedPtr msg)
{
  // Visualization
  vis_header_.stamp = msg->header.stamp;
  cv_bridge::CvImage image(vis_header_, sensor_msgs::image_encodings::RGB8);
  vis_.draw_new_image(image.image);

  // TODO(mdlopezme): Draw RPP Path
  if (!msg->tracked_objects.detections.empty()) {
    vis_.draw_opponent(image.image, msg->tracked_objects);
  }
  if (msg->rde_tlm.ttl_index) {
    vis_.draw_ttl(
      image.image,
      static_cast<ttl::TtlIndex>(msg->rde_tlm.ttl_index),
      TtlDrawMode::ALL);
  }
  if (msg->vehicle_cmd.flags) {
    vis_.draw_flags(
      image.image,
      std::make_shared<race_msgs::msg::VehicleCommand>(msg->vehicle_cmd));
  }
  vis_.draw_ego(image.image);
  vis_.draw_control(image.image, msg->rvc_tlm);
  vis_.draw_decision(
    image.image,
    msg->target_trajectory_cmd,
    msg->vks,
    static_cast<ttl::TtlIndex>(msg->rde_tlm.ttl_index));

  pub_vis_->publish(*image.toImageMsg());
}
}  // namespace race

RCLCPP_COMPONENTS_REGISTER_NODE(race::Visualization)
