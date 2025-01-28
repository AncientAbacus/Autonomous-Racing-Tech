// Copyright 2024 AI Racing Tech
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

#include <cmath>
#include "ghost_car/controller_plugin.hpp"

namespace race::ghost_car
{

class AutonomousController : public GhostCarController
{
public:
  bool configure() override
  {
    node().declare_parameter<std::vector<int>>("target_ttl_indexes");
    node().declare_parameter<std::vector<double>>("target_speeds");
    node().declare_parameter<std::vector<int>>("initial_wp_indexes");
    node().declare_parameter<std::string>("ttl_dir");
    ttl_tree_ = std::make_unique<race::ttl::TtlTree>(
      node().get_parameter(
        "ttl_dir").as_string().c_str());

    if (node().has_parameter("target_ttl_indexes")) {
      auto target_ttl_indexes = node().get_parameter("target_ttl_indexes").as_integer_array();
      ghost_car_count_ = target_ttl_indexes.size();
      for (int i = 0; i < ghost_car_count_; i++) {
        RCLCPP_WARN(node().get_logger(), "TTL Index: %d", target_ttl_indexes[i]);
        if (!ttl_tree_->is_ttl_index_valid(target_ttl_indexes[i])) {
          throw std::runtime_error("Invalid ttl index passed");
        }
        target_ttl_indexes_[i] =
          static_cast<race::ttl::TtlIndex>(target_ttl_indexes[i]);
      }
    } else {
      throw std::runtime_error("No target ttl indexes passed");
    }

    if (node().has_parameter("initial_wp_indexes")) {
      auto initial_wp_indexes = node().get_parameter("initial_wp_indexes").as_integer_array();
      for (int i = 0; i < ghost_car_count_; i++) {
        if (initial_wp_indexes[i] < 0 || static_cast<uint32_t>(initial_wp_indexes[i]) >=
          ttl_tree_->get_ttl(target_ttl_indexes_[i]).header.loop)
        {
          throw std::runtime_error("Invalid initial wp index passed");
        }
        auto wp = ttl_tree_->get_ttl(target_ttl_indexes_[i]).waypoints.at(initial_wp_indexes[i]);
        distances_[i] = wp.dist_to_sf_bwd;
        prev_distances_[i] = wp.dist_to_sf_bwd;
        latest_times_[i] = node().now();
      }
    } else {
      throw std::runtime_error("No initial wp indexes passed");
    }

    if (node().has_parameter("target_speeds")) {
      auto target_speeds = node().get_parameter("target_speeds").as_double_array();

      for (int i = 0; i < ghost_car_count_; i++) {
        if (target_speeds[i] < 0.0) {
          throw std::runtime_error("Invalid speed passed");
        }
        target_speeds_[i] = target_speeds[i] / 2.237;
      }
    } else {
      throw std::runtime_error("No target speeds passed");
    }

    RCLCPP_INFO(node().get_logger(), "Autonomous Controller configured");
    RCLCPP_INFO(node().get_logger(), "Number of cars: %d", ghost_car_count_);
    for (int i = 0; i < ghost_car_count_; i++) {
      RCLCPP_INFO(
        node().get_logger(), "Car %d: TTL Index: %d, Speed: %f", i,
        static_cast<int>(target_ttl_indexes_[i]),
        target_speeds_[i]);
    }

    node().declare_parameter<int>("ghost_car_count", ghost_car_count_);
    return true;
  }

  void updatePositionAndSpeed(
    race::ttl::Position & position, double & speed,
    double & yaw_angle, int index) override
  {
    rclcpp::Time t = node().now();
    double dt = (t - latest_times_[index]).seconds();
    distances_[index] += (target_speeds_[index] * dt);
    distances_[index] = std::fmod(
      distances_[index], ttl_tree_->get_ttl(
        target_ttl_indexes_[index]).header.total_distance);
    latest_times_[index] = t;
    auto wp_index = race::ttl::find_closest_waypoint_index(
      ttl_tree_->get_ttl(target_ttl_indexes_[index]), position);
    auto next_wp_index = race::ttl::inc_waypoint_index(
      ttl_tree_->get_ttl(target_ttl_indexes_[index]),
      wp_index);
    auto wp = ttl_tree_->get_ttl(target_ttl_indexes_[index]).waypoints.at(wp_index);
    auto next_wp = ttl_tree_->get_ttl(target_ttl_indexes_[index]).waypoints.at(next_wp_index);

    double min_dist = std::abs(
      std::fmod(
        (wp.dist_to_sf_bwd - distances_[index]),
        ttl_tree_->get_ttl(target_ttl_indexes_[index]).header.total_distance));
    double dist = std::abs(
      std::fmod(
        (next_wp.dist_to_sf_bwd - distances_[index]),
        ttl_tree_->get_ttl(target_ttl_indexes_[index]).header.total_distance));
    while (min_dist > dist || min_dist > 100.0) {
      min_dist = dist;
      wp_index = next_wp_index;
      next_wp_index = race::ttl::inc_waypoint_index(
        ttl_tree_->get_ttl(target_ttl_indexes_[index]),
        wp_index);
      wp = next_wp;
      next_wp = ttl_tree_->get_ttl(target_ttl_indexes_[index]).waypoints.at(next_wp_index);
      dist = std::abs(
        std::fmod(
          (next_wp.dist_to_sf_bwd - distances_[index]),
          ttl_tree_->get_ttl(target_ttl_indexes_[index]).header.total_distance));
    }
    auto fraction =
      abs(
      0.5 + (distances_[index] - wp.dist_to_sf_bwd) /
      (next_wp.dist_to_sf_bwd - wp.dist_to_sf_bwd));
    race::ttl::Waypoint interpolated_wp;
    auto prev_wp_index = race::ttl::dec_waypoint_index(
      ttl_tree_->get_ttl(target_ttl_indexes_[index]),
      wp_index);
    auto interpolated_position = race::ttl::interpolate_positions(
      ttl_tree_->get_ttl(target_ttl_indexes_[index]).waypoints.at(prev_wp_index).location,
      wp.location, fraction);
    ttl_tree_->get_projection(
      ttl_tree_->get_ttl(target_ttl_indexes_[index]).waypoints.at(prev_wp_index),
      wp, interpolated_wp, interpolated_position);
    position = interpolated_wp.location;
    speed = (distances_[index] - prev_distances_[index]) / dt;
    prev_distances_[index] = distances_[index];
    yaw_angle = interpolated_wp.target_yaw;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      if (param.get_name() == "target_ttl_indexes") {
        auto target_ttl_indexes = param.as_integer_array();
        for (int i = 0; i < ghost_car_count_; i++) {
          if (!ttl_tree_->is_ttl_index_valid(target_ttl_indexes[i])) {
            return false;
          }
        }
        for (int i = 0; i < ghost_car_count_; i++) {
          target_ttl_indexes_[i] =
            static_cast<race::ttl::TtlIndex>(target_ttl_indexes[i]);
        }
        return true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (param.get_name() == "target_speeds") {
        auto target_speeds = param.as_double_array();
        for (int i = 0; i < ghost_car_count_; i++) {
          if (target_speeds[i] < 0.0) {
            return false;
          }
        }
        for (int i = 0; i < ghost_car_count_; i++) {
          target_speeds_[i] = target_speeds[i] / 2.237;
        }
        return true;
      }
    }
    return false;
  }

private:
  // move to plugin
  std::unordered_map<int, double> target_speeds_;
  std::unordered_map<int, double> distances_;
  std::unordered_map<int, double> prev_distances_;
  std::unordered_map<int, race::ttl::TtlIndex> target_ttl_indexes_;
  std::unordered_map<int, rclcpp::Time> latest_times_;
  int ghost_car_count_;
  race::ttl::TtlTree::UniquePtr ttl_tree_;
};
}  // namespace race::ghost_car

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(race::ghost_car::AutonomousController, race::ghost_car::GhostCarController)
