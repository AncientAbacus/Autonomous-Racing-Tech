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


#include <fstream>
#include <chrono>
#include <memory>
#include <string>

#include "tf2_ros/transform_listener.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "polygon_transformer/polygon_transformer.hpp"

using std::chrono::duration;
using race::perception::PolygonTransformer;
using std::placeholders::_1;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Quaternion;
using tf2::doTransform;
namespace race
{
namespace perception
{
PolygonTransformer::PolygonTransformer(const rclcpp::NodeOptions & options)
: Node("polygon_transformer", options), tf_helper_(*this)
{
  std::string polygon_csv_path = declare_parameter<std::string>("polygon_csv");
  auto dt_ = declare_parameter("polygon_dt", 0.05);
  declare_parameter("frame_id", "base_link");
  update_polygon_timer_ = rclcpp::create_timer(
    this, get_clock(), duration<float>(dt_), [this] {
      update_polygon_cb();
    });
  read_polygon(polygon_csv_path);
  polygon_pub_ =
    create_publisher<Polygon>("pc_remover_polygon", rclcpp::QoS{1});
  if (declare_parameter("visualize_polygon", true)) {
    polygon_vis_pub_ = create_publisher<PolygonStamped>(
      "pc_remover_polygon_vis",
      rclcpp::QoS{1});
  }
}

void PolygonTransformer::read_polygon(const std::string & polygon_csv_path)
{
  enu_fixed_polygon_ = std::make_shared<Polygon>();
  RCLCPP_INFO(get_logger(), "Reading polygon from %s", polygon_csv_path.c_str());
  std::ifstream csv_stream(polygon_csv_path);
  if (!csv_stream.is_open()) {
    throw std::runtime_error("Could not open polygon CSV.");
  }
  std::string line, colname;
  int count = 0;
  while (std::getline(csv_stream, line)) {
    count++;
    std::stringstream ss(line);
    double coord[2] = {0, 0};
    for (int i = 0; i < 2; i++) {
      ss >> coord[i];
      if (ss.peek() == ',') {ss.ignore();}
    }
    auto coord_pt = Point32();
    coord_pt.x = static_cast<float>(coord[0]);
    coord_pt.y = static_cast<float>(coord[1]);
    coord_pt.z = 0.0;
    enu_fixed_polygon_->points.push_back(coord_pt);
  }
  csv_stream.close();
  RCLCPP_INFO(get_logger(), "Finished reading polygon.");
}

void PolygonTransformer::update_polygon_cb()
{
  auto tf = geometry_msgs::msg::TransformStamped();
  tf_helper_.lookup_transform("map", "base_link", tf, rclcpp::Time());

  auto poly = Polygon();
  poly.points.reserve(enu_fixed_polygon_->points.size());

  for (const auto & pt : enu_fixed_polygon_->points) {
    auto transformed_pose = point2pose(pt);
    tf2::doTransform(transformed_pose, transformed_pose, tf);
    auto transformed_pt = pose2point(transformed_pose);
    poly.points.push_back(transformed_pt);
  }
  polygon_pub_->publish(poly);

  if (get_parameter("visualize_polygon").as_bool()) {
    auto poly_vis = PolygonStamped();
    copy_polygon(poly, poly_vis.polygon);
    poly_vis.header.frame_id = get_parameter("frame_id").as_string();
    poly_vis.header.stamp = tf.header.stamp;
    polygon_vis_pub_->publish(poly_vis);
  }
}

PoseStamped PolygonTransformer::point2pose(const Point32 & pt)
{
  auto result = PoseStamped();
  result.pose.position.x = pt.x;
  result.pose.position.y = pt.y;
  result.pose.position.z = pt.z;
  return result;
}

Point32 PolygonTransformer::pose2point(const PoseStamped & pt)
{
  auto result = Point32();
  result.x = pt.pose.position.x;
  result.y = pt.pose.position.y;
  result.z = pt.pose.position.z;
  return result;
}

void PolygonTransformer::copy_polygon(
  const Polygon & src,
  Polygon & dst)
{
  dst.points.clear();
  dst.points.reserve(src.points.size());
  for (const auto & pt : src.points) {
    auto & new_pt = dst.points.emplace_back();
    new_pt.x = pt.x;
    new_pt.y = pt.y;
    new_pt.z = pt.z;
  }
}
}  // namespace perception
}  // namespace race

RCLCPP_COMPONENTS_REGISTER_NODE(race::perception::PolygonTransformer)
