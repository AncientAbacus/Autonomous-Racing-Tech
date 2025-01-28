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

#include "race_telemetry/visualizer.hpp"

#include <vector>
#include <string>
#include <algorithm>

#include "base_common/race_control.hpp"
#include "race_msgs/msg/vehicle_telemetry.hpp"

namespace race
{
TelemetryVisualizer::TelemetryVisualizer(rclcpp::Node * node, const char * ttl_dir)
: ttl_tree_(ttl_dir), tf_helper_(*node)
{
  // Let's start with some helper lambdas for declaring visualization parameters
  auto declare_cv_scalar = [&node](const char * param)
    {
      auto rgb = node->declare_parameter<std::vector<int64_t>>(param);
      return cv::Scalar(rgb[0], rgb[1], rgb[2]);
    };

  auto declare_cv_param =
    [&node, &declare_cv_scalar](const char * thickness_param, const char * rgb_param)
    {
      return CvParams{
      static_cast<uint32_t>(node->declare_parameter<int64_t>(thickness_param)),
      declare_cv_scalar(rgb_param)};
    };

  // Now we configure how to visualize

  const auto params_ttl = declare_cv_param("visualization.ttl.thickness", "visualization.ttl.rgb");
  const auto params_rpp = declare_cv_param("visualization.rpp.thickness", "visualization.rpp.rgb");
  const auto params_boundary = declare_cv_param(
    "visualization.boundary.thickness",
    "visualization.boundary.rgb");
  const auto params_opponent = declare_cv_param(
    "visualization.opponent.thickness",
    "visualization.opponent.rgb");
  const auto params_ego = EgoParams{
    declare_cv_param("visualization.ego.thickness", "visualization.ego.rgb"),
    node->declare_parameter<double>("visualization.ego.width"),
    node->declare_parameter<double>("visualization.ego.height"),
    node->declare_parameter<double>("visualization.ego.translation_x"),
    node->declare_parameter<double>("visualization.ego.translation_y"),
  };

  const auto params_txt = TextParams{
    node->declare_parameter<double>("visualization.text.ratio_start"),
    node->declare_parameter<double>("visualization.text.top_margin"),
    node->declare_parameter<double>("visualization.text.gap"),
    node->declare_parameter<double>("visualization.text.font_scale"),
    declare_cv_scalar("visualization.text.rgb"),
    declare_cv_scalar("visualization.text.rgb_highlight"),
  };

  config_ = VisualizationConfig{
    static_cast<size_t>(node->declare_parameter<int64_t>("visualization.pixel_per_m")),
    static_cast<size_t>(node->declare_parameter<int64_t>("visualization.image_w")),
    static_cast<size_t>(node->declare_parameter<int64_t>("visualization.image_h")),
    static_cast<size_t>(node->declare_parameter<int64_t>(
      "visualization.num_waypoint_to_visualize")),
    node->declare_parameter<double>("visualization.timeout_sec"),
    declare_cv_scalar("visualization.background_dark.rgb"),
    declare_cv_scalar("visualization.background_light.rgb"),
    params_ttl,
    params_rpp,
    params_boundary,
    params_opponent,
    params_ego,
    params_txt
  };
}

void TelemetryVisualizer::draw_new_image(cv::Mat & in)
{
  in = cv::Mat(config_.image_h, config_.image_w, CV_8UC3, config_.background_dark_rgb);
  cv::rectangle(
    in,
    cv::Point(config_.image_w * config_.txt.ratio_start * 0.97, 0),
    cv::Point(config_.image_w, config_.image_h),
    config_.background_light_rgb,
    cv::FILLED,
    cv::LINE_8);
}

void TelemetryVisualizer::draw_rpp_trajectory(cv::Mat & in, const nav_msgs::msg::Path & path)
{
  auto transform = geometry_msgs::msg::TransformStamped();
  tf_helper_.lookup_transform(path.header.frame_id, "base_link", transform, rclcpp::Time());
  const int num_pt = path.poses.size();
  std::vector<cv::Point> path_points;
  path_points.reserve(num_pt);
  for (const auto & pose : path.poses) {
    auto local_pose = geometry_msgs::msg::PoseStamped();
    tf2::doTransform(pose, local_pose, transform);
    path_points.push_back(
      to_image_coordinate(
        local_pose.pose.position.x,
        local_pose.pose.position.y
    ));
  }
  const auto pts = path_points.data();
  cv::polylines(in, &pts, &num_pt, 1, false, config_.rpp.rgb, config_.rpp.thickness, cv::LINE_AA);
}

void TelemetryVisualizer::draw_ttl(
  cv::Mat & in, const ttl::TtlIndex ttl_index,
  const TtlDrawMode & mode)
{
  // Extract ttl, left bound, and right bound into CV points
  auto transform = geometry_msgs::msg::TransformStamped();
  tf_helper_.lookup_transform("map", "base_link", transform, rclcpp::Time());
  auto inv_transform = geometry_msgs::msg::TransformStamped();
  tf_helper_.lookup_transform("base_link", "map", inv_transform, rclcpp::Time());
  const auto current_waypoint_index = ttl_tree_.find_closest_waypoint_index(
    ttl_index,
    race::ttl::Position{inv_transform.transform.translation.x,
      inv_transform.transform.translation.y});
  const auto & ttl = ttl_tree_.get_ttl(ttl_index);

  auto ttl_points = std::vector<cv::Point>(config_.num_waypoint_to_visualize);
  auto left_points = std::vector<cv::Point>(config_.num_waypoint_to_visualize);
  auto right_points = std::vector<cv::Point>(config_.num_waypoint_to_visualize);

  size_t wp_iter = current_waypoint_index;
  for (size_t i = 0; i < config_.num_waypoint_to_visualize / 2; i++) {
    wp_iter = ttl::dec_waypoint_index(ttl, wp_iter);
  }
  auto ttl_iter = ttl_points.begin();
  auto left_iter = left_points.begin();
  auto right_iter = right_points.begin();

  while (ttl_iter != ttl_points.end()) {
    const auto & wp = ttl.waypoints.at(wp_iter);
    ttl::Position local_ttl_position, local_left_position, local_right_position;
    TransformHelper::do_transform(wp.location, local_ttl_position, transform);
    TransformHelper::do_transform(wp.left_bound, local_left_position, transform);
    TransformHelper::do_transform(wp.right_bound, local_right_position, transform);
    *ttl_iter = to_image_coordinate(local_ttl_position.x, local_ttl_position.y);
    *left_iter = to_image_coordinate(local_left_position.x, local_left_position.y);
    *right_iter = to_image_coordinate(local_right_position.x, local_right_position.y);

    wp_iter = ttl::inc_waypoint_index(ttl, wp_iter);
    ttl_iter++;
    left_iter++;
    right_iter++;
  }

  // Draw the lines
  const auto draw_polylines =
    [&in](const std::vector<cv::Point> & path_points, const CvParams & params)
    {
      const auto pts = path_points.data();
      const int num_pt = path_points.size();
      cv::polylines(in, &pts, &num_pt, 1, false, params.rgb, params.thickness, cv::LINE_AA);
    };

  if (mode == TtlDrawMode::ALL || mode == TtlDrawMode::PATH_ONLY) {
    draw_polylines(ttl_points, config_.ttl);
  }
  if (mode == TtlDrawMode::ALL || mode == TtlDrawMode::BOUNDS_ONLY) {
    draw_polylines(left_points, config_.boundary);
    draw_polylines(right_points, config_.boundary);
  }
}

void TelemetryVisualizer::draw_opponent(
  cv::Mat & in,
  const vision_msgs::msg::Detection3DArray & opponent)
{
  auto transform = geometry_msgs::msg::TransformStamped();
  tf_helper_.lookup_transform("map", "base_link", transform, rclcpp::Time());

  for (const auto & obj : opponent.detections) {
    auto transform = geometry_msgs::msg::TransformStamped();
    tf_helper_.lookup_transform("map", "base_link", transform, rclcpp::Time());

    for (const auto & obj : opponent.detections) {
      (void)obj;  // suppress unused variable warning
      geometry_msgs::msg::PoseStamped global_opponent_pose, local_opponent_pose;
      global_opponent_pose.pose = obj.bbox.center;
      tf2::doTransform(global_opponent_pose, local_opponent_pose, transform);
      cv::Point center = to_image_coordinate(
        local_opponent_pose.pose.position.x,
        local_opponent_pose.pose.position.y);
      cv::Size2f size(obj.bbox.size.x * config_.pixel_per_m,
        obj.bbox.size.y * config_.pixel_per_m);
      float angle = TransformHelper::heading_from_quaternion(local_opponent_pose.pose.orientation);
      cv::RotatedRect bbox(center, size, angle);
      std::vector<cv::Point2f> vertices_f(4);
      bbox.points(vertices_f.data());
      std::vector<cv::Point> vertices(vertices_f.begin(), vertices_f.end());
      auto pts = vertices.data();
      int num_pt = 4;
      cv::polylines(
        in, &pts, &num_pt, 1, true, config_.opponent.rgb, config_.opponent.thickness,
        cv::LINE_AA);
    }
  }
}

void TelemetryVisualizer::draw_ego(cv::Mat & in)
{
  // Draw ego as a rectangle
  ttl::Position p_tl{-config_.ego.height / 2.0,
    -config_.ego.width / 2.0};  // top left corner
  ttl::Position p_br{config_.ego.height / 2.0,
    config_.ego.width / 2.0};  // bottom right corner
  cv::rectangle(
    in,
    to_image_coordinate(p_tl.x, p_tl.y),
    to_image_coordinate(p_br.x, p_br.y),
    config_.ego.vis.rgb, config_.ego.vis.thickness, cv::LINE_AA);
  ttl::Position p_front{config_.ego.height / 3.0, config_.ego.width / 2.0};
  cv::rectangle(
    in,
    to_image_coordinate(p_tl.x, p_tl.y),
    to_image_coordinate(p_front.x, p_front.y),
    config_.ego.vis.rgb, config_.ego.vis.thickness, cv::LINE_AA);
}

void TelemetryVisualizer::draw_flags(
  cv::Mat & in,
  const race_msgs::msg::VehicleCommand::SharedPtr & msg)
{
  // Vehicle Signal

  bool fill_color = true;
  auto flags = race::get_flags(*msg);

  cv::Scalar flag_color_vehicle_signal = config_.background_light_rgb;
  if (flags.has_purple_flag()) {
    flag_color_vehicle_signal = cv::Scalar(128, 0, 128);
  } else if (flags.has_black_flag()) {
    flag_color_vehicle_signal = cv::Scalar(0, 0, 0);
  } else if (flags.has_checkered_flag()) {
    flag_color_vehicle_signal = cv::Scalar(69, 69, 69);
  } else if (flags.has_defender_flag()) {
    flag_color_vehicle_signal = cv::Scalar(0, 0, 255);
    fill_color = false;
  } else if (flags.has_attacker_flag()) {
    flag_color_vehicle_signal = cv::Scalar(255, 0, 0);
    fill_color = false;
  } else if (flags.has_orange_flag()) {
    flag_color_vehicle_signal = cv::Scalar(255, 165, 0);
  } else if (flags.has_yellow_flag()) {
    flag_color_vehicle_signal = cv::Scalar(255, 255, 0);
  } else if (flags.has_stop_flag()) {
    flag_color_vehicle_signal = cv::Scalar(255, 0, 0);
  }
  TelemetryVisualizer::draw_flag_helper(
    in,
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + 1 * config_.txt.gap)),
    cv::Point(
      config_.image_w * (config_.txt.ratio_start + 0.1),
      config_.image_h * (config_.txt.top_margin + 3 * config_.txt.gap)),
    flag_color_vehicle_signal, fill_color, "VEC", "");

  // Track Condition

  fill_color = true;
  std::string secondary_text = "";

  cv::Scalar flag_color_track_condition = config_.background_light_rgb;
  if (flags.has_red_flag()) {
    flag_color_track_condition = cv::Scalar(255, 0, 0);
  } else if (flags.has_fcy_flag()) {
    flag_color_track_condition = cv::Scalar(255, 255, 0);
  } else if (flags.has_green_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
  } else if (flags.has_blue_flag()) {
    flag_color_track_condition = cv::Scalar(0, 0, 255);
  } else if (flags.has_wgreen_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    fill_color = false;
  } else if (flags.has_g80_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "80";
  } else if (flags.has_g100_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "100";
  } else if (flags.has_g120_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "120";
  } else if (flags.has_g130_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "130";
  } else if (flags.has_g140_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "140";
  } else if (flags.has_g145_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "145";
  } else if (flags.has_g150_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "150";
  } else if (flags.has_g155_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "155";
  } else if (flags.has_g160_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "160";
  } else if (flags.has_g165_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "165";
  } else if (flags.has_g170_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "170";
  } else if (flags.has_g175_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "175";
  } else if (flags.has_g180_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "180";
  } else if (flags.has_g185_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "185";
  } else if (flags.has_g190_flag()) {
    flag_color_track_condition = cv::Scalar(0, 128, 0);
    secondary_text = "190";
  }

  TelemetryVisualizer::draw_flag_helper(
    in,
    cv::Point(
      config_.image_w * (config_.txt.ratio_start * 1.2),
      config_.image_h * (config_.txt.top_margin + 1 * config_.txt.gap)),
    cv::Point(
      config_.image_w * (config_.txt.ratio_start * 1.2 + 0.1),
      config_.image_h * (config_.txt.top_margin + 3 * config_.txt.gap)),
    flag_color_track_condition, fill_color, "TRK", secondary_text);

  // Round Target Speed
  int level = 6;
  cv::putText(
    in,
    "RNG TGT",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);
  cv::putText(
    in,
    std::to_string(msg->round_target_speed).substr(0, 5),
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);
}

void TelemetryVisualizer::draw_flag_helper(
  cv::Mat & in, cv::Point tl, cv::Point br, cv::Scalar color, bool fill_color, std::string text,
  std::string secondary_text)
{
  double flag_boundary_thickness = std::max(1, static_cast<int>(config_.image_h / 300.0));
  if (fill_color) {
    cv::rectangle(in, tl, br, color, cv::FILLED, cv::LINE_AA);
  } else {
    cv::rectangle(in, tl, br, color, flag_boundary_thickness, cv::LINE_AA);
  }

  cv::putText(
    in,
    text,
    cv::Point(tl.x, tl.y - config_.image_h * config_.txt.gap * 0.33),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);

  cv::putText(
    in,
    secondary_text,
    cv::Point(tl.x, tl.y + config_.image_h * config_.txt.gap * 1.33),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);
}

void TelemetryVisualizer::draw_decision(
  cv::Mat & in,
  const race_msgs::msg::TargetTrajectoryCommand & msg,
  const race_msgs::msg::VehicleKinematicState & msg_vks,
  const ttl::TtlIndex ttl_index)
{
  int level = 4;
  // Cruise or Following
  std::string strategy_type = "";
  if (msg.strategy_type.strategy_type == race_msgs::msg::StrategyType::CRUISE_CONTROL) {
    strategy_type = "CRUISE";
  } else if (msg.strategy_type.strategy_type == race_msgs::msg::StrategyType::FOLLOW_MODE) {
    strategy_type = "FOLLOWING";
  } else {
    strategy_type = "UNKNOWN";
  }

  cv::putText(
    in,
    "STRATEGY",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);
  cv::putText(
    in,
    strategy_type,
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);

  // Leave two-line space for "Round Target Speed"
  level++;
  level++;

  // Target Speed
  cv::putText(
    in,
    "SPD LMT",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);
  cv::putText(
    in,
    std::to_string(msg.target_speed * race::MS_TO_MPH).substr(0, 5),
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);

  // Current Speed
  cv::putText(
    in,
    "CRT SPD",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);
  cv::putText(
    in,
    std::to_string(msg_vks.speed_mps * race::MS_TO_MPH).substr(0, 5),
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);

  // Target Gap
  cv::putText(
    in,
    "TGT GAP",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);
  cv::putText(
    in,
    std::to_string(msg.target_gap).substr(0, 5),
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);

  // Current Gap
  cv::putText(
    in,
    "CRT GAP",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);
  cv::putText(
    in,
    std::to_string(msg.rival_car_gap).substr(0, 5),
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);

  // TTL Index
  cv::putText(
    in,
    "TTL IDX",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);
  cv::putText(
    in,
    std::to_string(static_cast<int16_t>(ttl_index)),
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb_highlight, 1, cv::LINE_AA);
}

double TelemetryVisualizer::get_target_yaw(
  const race_msgs::msg::RvcTelemetry & msg,
  const ttl::TtlIndex ttl_index
)
{
  race::ttl::Position lookahead_pos{msg.lookahead_x, msg.lookahead_y};
  auto lookahead_wp_index = ttl_tree_.find_closest_waypoint_index(ttl_index, lookahead_pos);
  return ttl_tree_.get_ttl(ttl_index).waypoints.at(lookahead_wp_index).target_yaw;
}


void TelemetryVisualizer::draw_control(cv::Mat & in, const race_msgs::msg::RvcTelemetry & msg)
{
  int level = 18;
  // Lateral Error
  cv::putText(
    in,
    "LAT ERR",
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);
  cv::putText(
    in,
    std::to_string(msg.lateral_error).substr(0, 5),
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level++ *config_.txt.gap)),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, config_.txt.rgb, 1, cv::LINE_AA);

  // Break
  TelemetryVisualizer::draw_bar_chart_helper(
    in,
    cv::Point(
      config_.image_w * config_.txt.ratio_start,
      config_.image_h * (config_.txt.top_margin + level * config_.txt.gap + 0.02)),
    config_.txt.rgb_highlight,
    static_cast<double>(msg.brake_cmd) / 2200.0, "BRK");

  // Throttle
  TelemetryVisualizer::draw_bar_chart_helper(
    in,
    cv::Point(
      config_.image_w * config_.txt.ratio_start * 1.2,
      config_.image_h * (config_.txt.top_margin + level * config_.txt.gap + 0.02)),
    config_.txt.rgb_highlight,
    static_cast<double>(msg.throttle_cmd) / 100.0, "THR");
}


void TelemetryVisualizer::draw_bar_chart_helper(
  cv::Mat & in, cv::Point tl_text, cv::Scalar color, double bar_height, std::string text)
{
  // Text: break/throttle, and it's value
  cv::putText(
    in,
    text,
    cv::Point(tl_text.x, tl_text.y - config_.image_h * config_.txt.gap * 0.33),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, color, 1, cv::LINE_AA);

  cv::putText(
    in,
    std::to_string(bar_height * 100).substr(0, 3),
    cv::Point(tl_text.x, tl_text.y + config_.image_h * config_.txt.gap * 0.67),
    cv::FONT_HERSHEY_PLAIN, config_.txt.font_scale, color, 1, cv::LINE_AA);

  // Draw bar chart
  cv::Point tl(
    tl_text.x,
    tl_text.y + config_.image_h * config_.txt.gap);  // add space for one more line text

  // ratio of bar chart height, width
  double ratio_h = config_.txt.gap * 3;
  double ratio_w = 0.1;
  double chart_boundary_thickness = std::max(1, static_cast<int>(config_.image_h / 300.0));
  cv::Point br(
    tl.x + config_.image_w * ratio_w,
    tl.y + config_.image_h * ratio_h);

  // draw bar chart boundary
  cv::rectangle(in, tl, br, color, chart_boundary_thickness, cv::LINE_AA);

  // fill bar chart
  cv::rectangle(
    in,
    cv::Point(
      tl.x,
      tl.y + config_.image_h * ratio_h * (1 - bar_height)),
    br, color, cv::FILLED, cv::LINE_AA);
}

cv::Point TelemetryVisualizer::to_image_coordinate(const double & x, const double & y)
{
  return cv::Point(
    -y * config_.pixel_per_m + config_.image_w / 3.0,
    -x * config_.pixel_per_m + config_.image_h / 2.0
  );
}
}  // namespace race
