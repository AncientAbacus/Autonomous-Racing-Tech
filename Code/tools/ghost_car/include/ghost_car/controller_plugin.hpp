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


/**
 * @file controller_plugin.hpp
 * @author Wayne Lai (wquanlai@gmail.com)
 * @brief plugin interface
 * @version 0.1
 * @date 2024-02-12
 *
 * @copyright Copyright 2024 AI Racing Tech
 *
 */

#ifndef GHOST_CAR__CONTROLLER_PLUGIN_HPP_
#define GHOST_CAR__CONTROLLER_PLUGIN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ttl.hpp"
#include "ttl_tree.hpp"
#include "vehicle_model/vehicle_model_config.hpp"
#include "vehicle_model/vehicle_model.hpp"
#include "vehicle_model/lookup.hpp"

namespace race::ghost_car
{
class GhostCarController
{
public:
  virtual ~GhostCarController() = default;
  virtual void updatePositionAndSpeed(
    race::ttl::Position & position, double & speed,
    double & yaw_angle, int index) = 0;
  /**
  * @brief Initialize plugin, saving pointers to vehicle model and
  * parent node. Do not override unless needed.
  *
  * @param node Pointer to the parent node for logging, timing, etc.
  * @param model Pointer to the vehicle model
  */
  virtual void initialize(rclcpp::Node * node, race::vehicle_model::VehicleModel::SharedPtr model)
  {
    node_ = node;
    m_model_ = model;
  }

  /**
  * @brief Declare ROS params, pubs, and subs, and initialize the
  * plugin here
  *
  * @return If initialization is successful
  */
  virtual bool configure() {return true;}

  // when there is changes onto ghost car params
  virtual bool update_param(const rclcpp::Parameter & param)
  {
    (void) param;
    return false;
  }

protected:
  rclcpp::Node & node() {return *node_;}
  race::vehicle_model::VehicleModel & model() {return *m_model_;}

private:
  rclcpp::Node * node_{};
  race::vehicle_model::VehicleModel::SharedPtr m_model_{};
};

}  // namespace race::ghost_car

#endif  // GHOST_CAR__CONTROLLER_PLUGIN_HPP_
