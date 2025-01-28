// Copyright 2022 Siddharth Saha
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

#ifndef GHOST_CAR__GHOST_CAR_NODE_HPP_
#define GHOST_CAR__GHOST_CAR_NODE_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <vector>
#include <memory>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "ttl.hpp"
#include "ttl_tree.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pluginlib/class_loader.hpp"
#include "vehicle_model/vehicle_model.hpp"
#include "vehicle_model/vehicle_model_config.hpp"
#include "vehicle_model/ros_param_loader.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ghost_car/controller_plugin.hpp"

namespace race::ghost_car
{
class GhostCar : public rclcpp::Node
{
public:
  explicit GhostCar(const rclcpp::NodeOptions & options);

private:
  // Helper functions
  void step();

  // controller
  pluginlib::ClassLoader<race::ghost_car::GhostCarController> controller_loader_;
  std::shared_ptr<race::ghost_car::GhostCarController> controller_;

  // Publishers
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr pub_ghost_car_;

  // vehicle model
  race::vehicle_model::VehicleModel::SharedPtr m_model_;

  // Position
  std::unordered_map<int, race::ttl::Position> positions_;

  // Timing
  double dt_{};
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::Time latest_time_;

  // Parameters
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Number of ghost cars
  int ghost_car_count_;
  int ghost_car_num_to_publish_;
};
}  // namespace race::ghost_car

#endif  // GHOST_CAR__GHOST_CAR_NODE_HPP_
