// Copyright 2023 Haoru Xue
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

#ifndef ART_MPC_INTERFACE__ART_MPC_INTERFACE_NODE_HPP_
#define ART_MPC_INTERFACE__ART_MPC_INTERFACE_NODE_HPP_

#include <memory>
#include <vector>

#include <casadi/casadi.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mpclab_msgs/msg/vehicle_state_msg.hpp>
#include <mpclab_msgs/msg/vehicle_actuation_msg.hpp>
#include <mpclab_msgs/msg/steering_report.hpp>
#include <lmpc_msgs/msg/trajectory_command.hpp>
#include <race_msgs/msg/vehicle_control_command.hpp>
#include <race_msgs/msg/engine_report.hpp>
#include <race_msgs/msg/vehicle_kinematic_state.hpp>
#include <lmpc_transform_helper/lmpc_transform_helper.hpp>
#include <single_track_planar_model/single_track_planar_model.hpp>
#include <racing_trajectory/racing_trajectory.hpp>

#include <race_msgs/msg/target_trajectory_command.hpp>
#include <race_msgs/msg/steering_report.hpp>

#include "art_mpc_interface/art_mpc_interface_config.hpp"

namespace lmpc
{
namespace interface
{
namespace art_mpc_interface
{
using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;

using lmpc::vehicle_model::racing_trajectory::RacingTrajectory;
using lmpc::vehicle_model::single_track_planar_model::SingleTrackPlanarModel;

class ARTMPCInterfaceNode : public rclcpp::Node
{
public:
  explicit ARTMPCInterfaceNode(const rclcpp::NodeOptions & options);

protected:
  ARTMPCInterfaceConfig::SharedPtr config_ {};
  SingleTrackPlanarModel::SharedPtr model_ {};
  uint64_t sim_step_ {0};
  uint64_t lap_count_ {0};
  tf2::Transform cg_to_baselink_ {};
  std::vector<double> x_;

  utils::TransformHelper tf_helper_;

  // message io
  PolygonStamped::SharedPtr vehicle_polygon_msg_ {};
  mpclab_msgs::msg::VehicleStateMsg::SharedPtr vehicle_state_msg_ {};
  mpclab_msgs::msg::VehicleActuationMsg::SharedPtr vehicle_actuation_msg_ {};
  TransformStamped::SharedPtr map_to_baselink_msg_ {};
  race_msgs::msg::VehicleKinematicState::SharedPtr vks_msg_ {};


  // publishers (to controller)
  rclcpp::Publisher<mpclab_msgs::msg::VehicleStateMsg>::SharedPtr vehicle_state_pub_;
  rclcpp::Publisher<lmpc_msgs::msg::TrajectoryCommand>::SharedPtr tc_pub_;
  rclcpp::Publisher<mpclab_msgs::msg::SteeringReport>::SharedPtr sr_pub_;


  // publishers (to visualization)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_pub_;

  // publishers (to art stack)
  rclcpp::Publisher<race_msgs::msg::VehicleControlCommand>::SharedPtr vehicle_control_command_pub_;

  // subscribers (from controller)
  rclcpp::Subscription<mpclab_msgs::msg::VehicleActuationMsg>::SharedPtr vehicle_actuation_sub_;

  // subscribers (from art stack)
  rclcpp::Subscription<race_msgs::msg::VehicleKinematicState>::SharedPtr vks_sub_;
  rclcpp::Subscription<race_msgs::msg::EngineReport>::SharedPtr engine_report_sub_;
  rclcpp::Subscription<race_msgs::msg::TargetTrajectoryCommand>::SharedPtr ttl_sub_;
  rclcpp::Subscription<race_msgs::msg::SteeringReport>::SharedPtr steering_report_sub_;


  // timers
  // a slow rate timer to visualize track boundaries and abscissa
  rclcpp::TimerBase::SharedPtr static_vis_timer_;

  // callbacks
  void on_actuation(mpclab_msgs::msg::VehicleActuationMsg::SharedPtr msg);
  void on_vks(race_msgs::msg::VehicleKinematicState::SharedPtr msg);
  void on_engine_report(race_msgs::msg::EngineReport::SharedPtr msg);
  void on_ttl(race_msgs::msg::TargetTrajectoryCommand::SharedPtr msg);
  void on_sr(race_msgs::msg::SteeringReport::SharedPtr msg);
  void on_state_update();

  // helper functions
  void update_vehicle_state_msg(
    const BodyVelocity2D & v_body,
    const Pose2D & global_pose);
};
}  // namespace art_mpc_interface
}  // namespace interface
}  // namespace lmpc
#endif  // ART_MPC_INTERFACE__ART_MPC_INTERFACE_NODE_HPP_
