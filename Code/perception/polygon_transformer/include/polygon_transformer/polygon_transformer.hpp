// Copyright 2022 Haoru Xue
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POLYGON_TRANSFORMER__POLYGON_TRANSFORMER_HPP_
#define POLYGON_TRANSFORMER__POLYGON_TRANSFORMER_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "transform_helper/transform_helper.hpp"
#include "ttl.hpp"


using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;
using race::ttl::to_enu;
using race::ttl::GpsPosition;
using race::ttl::Position;
using tf2_ros::TransformBroadcaster;

namespace race
{
namespace perception
{

class PolygonTransformer : public rclcpp::Node
{
public:
  explicit PolygonTransformer(const rclcpp::NodeOptions & options);

private:
  void update_polygon_cb();
  void read_polygon(const std::string & polygon_csv_path);
  void copy_polygon(const Polygon & src, Polygon & dst);
  PoseStamped point2pose(const Point32 & pt);
  Point32 pose2point(const PoseStamped & pt);

  rclcpp::Publisher<Polygon>::SharedPtr polygon_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr polygon_vis_pub_;
  rclcpp::TimerBase::SharedPtr update_polygon_timer_;

  Polygon::SharedPtr enu_fixed_polygon_;

  TransformHelper tf_helper_;
};
}  // namespace perception
}  // namespace race

#endif  // POLYGON_TRANSFORMER__POLYGON_TRANSFORMER_HPP_
