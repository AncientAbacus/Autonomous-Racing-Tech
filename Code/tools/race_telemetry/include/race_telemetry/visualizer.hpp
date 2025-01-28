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
//
// Creates image visualizations of telemetry data

#ifndef RACE_TELEMETRY__VISUALIZER_HPP_
#define RACE_TELEMETRY__VISUALIZER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/path.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"

#include "ttl_tree.hpp"
#include "transform_helper/transform_helper.hpp"
#include "base_common/race_control.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/vehicle_telemetry.hpp"

namespace race
{
const double MS_TO_MPH = 2.23694;
struct CvParams
{
  uint32_t thickness;
  cv::Scalar rgb;
};

struct EgoParams
{
  CvParams vis;
  double width;
  double height;
  double translation_x;
  double translation_y;
};

struct TextParams
{
  double ratio_start;
  double top_margin;
  double gap;
  double font_scale;
  cv::Scalar rgb;
  cv::Scalar rgb_highlight;
};

struct VisualizationConfig
{
  size_t pixel_per_m;
  size_t image_w;
  size_t image_h;
  size_t num_waypoint_to_visualize;
  double timeout_sec;
  cv::Scalar background_dark_rgb;
  cv::Scalar background_light_rgb;
  CvParams ttl;
  CvParams rpp;
  CvParams boundary;
  CvParams opponent;
  EgoParams ego;
  TextParams txt;
};

enum TtlDrawMode
{
  ALL,
  BOUNDS_ONLY,
  PATH_ONLY
};

class TelemetryVisualizer
{
public:
  TelemetryVisualizer(rclcpp::Node * node, const char * ttl_dir);

  VisualizationConfig & config();

  void draw_new_image(cv::Mat & in);

  void draw_rpp_trajectory(
    cv::Mat & in,
    const nav_msgs::msg::Path & path);
  void draw_ttl(
    cv::Mat & in, const ttl::TtlIndex ttl_index,
    const TtlDrawMode & mode);
  void draw_opponent(
    cv::Mat & in,
    const vision_msgs::msg::Detection3DArray & opponent);
  void draw_ego(cv::Mat & in);
  void draw_flags(
    cv::Mat & in,
    const race_msgs::msg::VehicleCommand::SharedPtr & msg);
  void draw_decision(
    cv::Mat & in,
    const race_msgs::msg::TargetTrajectoryCommand & msg,
    const race_msgs::msg::VehicleKinematicState & msg_vks,
    const ttl::TtlIndex ttl_index);
  void draw_control(
    cv::Mat & in,
    const race_msgs::msg::RvcTelemetry & msg);
  double get_target_yaw(
    const race_msgs::msg::RvcTelemetry & msg,
    const ttl::TtlIndex ttl_index
  );

protected:
  VisualizationConfig config_;
  ttl::TtlTree ttl_tree_;
  TransformHelper tf_helper_;
  double font_scale;  // fontScale of cv::Puttext()
  int level = 1;

  /*
  Rotate image by 90 degree in clockwise, and translate image by half width, half height.
  Note:
  point (x, y) in natrual coordinate is (x, -y) in opencv image coordinate;
  so rotate by 90 degre -> (-y, -x).
  */
  cv::Point to_image_coordinate(const double & x, const double & y);

  void draw_flag_helper(
    cv::Mat & in, cv::Point tl, cv::Point br, cv::Scalar color, bool fill_color, std::string text,
    std::string secondary_text);
  void draw_bar_chart_helper(
    cv::Mat & in, cv::Point tl, cv::Scalar color, double bar_height, std::string text);

private:
  race::ttl::TtlIndex current_ttl_index_;
};
}  // namespace race

#endif  // RACE_TELEMETRY__VISUALIZER_HPP_
