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

#include <memory>
#include <algorithm>

#include <base_vehicle_model/ros_param_loader.hpp>
#include <single_track_planar_model/ros_param_loader.hpp>

#include "art_mpc_interface/art_mpc_interface_node.hpp"
#include "art_mpc_interface/ros_param_loader.hpp"

namespace lmpc
{
namespace interface
{
namespace art_mpc_interface
{
ARTMPCInterfaceNode::ARTMPCInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("art_mpc_interface_node", options),
  config_(lmpc::interface::art_mpc_interface::load_parameters(this)),
  model_(std::make_shared<SingleTrackPlanarModel>(
      lmpc::vehicle_model::base_vehicle_model::
      load_parameters(this), lmpc::vehicle_model::single_track_planar_model::load_parameters(
        this))),
  sim_step_(0),
  lap_count_(0),
  x_(model_->nx(), 0.0),
  tf_helper_(*this)
{
  // build the cg to base_link transform
  const auto & chassis_config = *(model_->get_base_config().chassis_config);
  const auto & lr = chassis_config.wheel_base * chassis_config.cg_ratio;
  const auto & lf = chassis_config.wheel_base - lr;
  cg_to_baselink_.setOrigin(tf2::Vector3(-1.0 * lr, 0.0, 0.0));
  cg_to_baselink_.setRotation(utils::TransformHelper::quaternion_from_heading(0.0));

  // initialize vehicle state message
  vehicle_state_msg_ = std::make_shared<mpclab_msgs::msg::VehicleStateMsg>();

  // initialize the map to base_link message
  if (config_->publish_tf) {
    map_to_baselink_msg_ = std::make_shared<TransformStamped>();
    map_to_baselink_msg_->header.frame_id = "map";
    map_to_baselink_msg_->child_frame_id = "base_link";

    // tf2::Transform map_to_cg;
    // map_to_cg.setOrigin(tf2::Vector3(global_pose.position.x, global_pose.position.y, 0.0));
    // map_to_cg.setRotation(utils::TransformHelper::quaternion_from_heading(global_pose.yaw));
    // map_to_baselink_msg_->transform = tf2::toMsg(map_to_cg);
  }

  // initialize state publishers
  vehicle_state_pub_ = this->create_publisher<mpclab_msgs::msg::VehicleStateMsg>(
    "vehicle_state", 1);
  tc_pub_ = this->create_publisher<lmpc_msgs::msg::TrajectoryCommand>(
    "lmpc_trajectory_command", 1);
  sr_pub_ = this->create_publisher<mpclab_msgs::msg::SteeringReport>(
    "mpc_steering_report", 1);

  // initialize art publishers
  vehicle_control_command_pub_ = this->create_publisher<race_msgs::msg::VehicleControlCommand>(
    "raw_cmd", 1);

  // initialize odom publishers
  vehicle_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "vehicle_odom", 1);

  // initialize subscribers
  vehicle_actuation_sub_ = this->create_subscription<mpclab_msgs::msg::VehicleActuationMsg>(
    "vehicle_actuation", 1,
    std::bind(&ARTMPCInterfaceNode::on_actuation, this, std::placeholders::_1));
  vks_sub_ = this->create_subscription<race_msgs::msg::VehicleKinematicState>(
    "vehicle_kinematic_state", rclcpp::SensorDataQoS(),
    std::bind(&ARTMPCInterfaceNode::on_vks, this, std::placeholders::_1));
  engine_report_sub_ = this->create_subscription<race_msgs::msg::EngineReport>(
    "engine_report", rclcpp::SensorDataQoS(),
    std::bind(&ARTMPCInterfaceNode::on_engine_report, this, std::placeholders::_1));
  ttl_sub_ = this->create_subscription<race_msgs::msg::TargetTrajectoryCommand>(
    "target_trajectory_command", rclcpp::SensorDataQoS(),
    std::bind(&ARTMPCInterfaceNode::on_ttl, this, std::placeholders::_1));
  steering_report_sub_ = this->create_subscription<race_msgs::msg::SteeringReport>(
    "steering_report", rclcpp::SensorDataQoS(),
    std::bind(&ARTMPCInterfaceNode::on_sr, this, std::placeholders::_1));
}

void ARTMPCInterfaceNode::on_actuation(mpclab_msgs::msg::VehicleActuationMsg::SharedPtr msg)
{
  vehicle_actuation_msg_ = msg;
  race_msgs::msg::VehicleControlCommand control_command;
  control_command.stamp = msg->header.stamp;
  control_command.steering_cmd = msg->u_steer * config_->steering_scale;

  const auto zero_throttle_force = model_->calc_drive_force(0.0);
  if (zero_throttle_force > msg->u_a * 1000.0) {
    control_command.accelerator_cmd = 0.0;
    control_command.brake_cmd = model_->calc_brake(msg->u_a * 1000.0 - zero_throttle_force);
  } else {
    control_command.accelerator_cmd = model_->calc_throttle(msg->u_a * 1000.0);
    control_command.brake_cmd = 0.0;
  }
  vehicle_control_command_pub_->publish(control_command);
}

void ARTMPCInterfaceNode::on_vks(race_msgs::msg::VehicleKinematicState::SharedPtr msg)
{
  vks_msg_ = msg;
  on_state_update();
}

void ARTMPCInterfaceNode::on_engine_report(race_msgs::msg::EngineReport::SharedPtr msg)
{
  model_->get_state().engine_rpm = msg->engine_rpm;
  model_->get_state().gear = msg->current_gear;
}

void ARTMPCInterfaceNode::on_ttl(race_msgs::msg::TargetTrajectoryCommand::SharedPtr msg)
{
  lmpc_msgs::msg::TrajectoryCommand tc_msg;
  tc_msg.header.stamp = msg->stamp;
  tc_msg.speed_limit = std::max(msg->target_speed, config_->min_vx);
  tc_msg.trajectory_index = msg->current_ttl_index;
  tc_msg.velocity_profile_scale = msg->target_waypoint_scale;
  tc_msg.target_gap = msg->target_gap;
  tc_msg.rival_car_gap = msg->rival_car_gap;
  tc_msg.rival_car_speed = msg->rival_car_speed;
  lmpc_msgs::msg::BehaviorStratergy behavior_strategy;
  race_msgs::msg::StrategyType strategy = msg->strategy_type;
  behavior_strategy.follow_distance = msg->target_gap;
  behavior_strategy.behavior_stratergy = strategy.strategy_type;
  tc_msg.behavior_stratergy = behavior_strategy;
  tc_pub_->publish(tc_msg);
}

void ARTMPCInterfaceNode::on_sr(race_msgs::msg::SteeringReport::SharedPtr msg)
{
  mpclab_msgs::msg::SteeringReport sr_msg;
  sr_msg.steer_angle = msg->front_wheel_angle_rad;
  sr_msg.steer_rate = msg->primary_steering_angular_rate;
  sr_pub_->publish(sr_msg);
}


void ARTMPCInterfaceNode::update_vehicle_state_msg(
  const BodyVelocity2D & v_body,
  const Pose2D & global_pose)
{
  // calculate body velocity
  auto vb = v_body;
  // MPC needs a min positive vx to work
  vb.x = std::max(vb.x, config_->min_vx);

  // build the updated state message
  // TODO(haoru): t?
  vehicle_state_msg_->t = now().seconds();
  vehicle_state_msg_->x.x = global_pose.position.x;
  vehicle_state_msg_->x.y = global_pose.position.y;
  vehicle_state_msg_->e.psi = global_pose.yaw;
  // TODO(haoru): populate body accel
  vehicle_state_msg_->v.v_long = vb.x;
  vehicle_state_msg_->v.v_tran = vb.y;
  vehicle_state_msg_->w.w_psi = vb.v_yaw;

  if (vehicle_actuation_msg_) {
    vehicle_state_msg_->u = *vehicle_actuation_msg_;
  } else {
    vehicle_state_msg_->u.u_a = 0.0;
    vehicle_state_msg_->u.u_steer = 0.0;
  }
}

void ARTMPCInterfaceNode::on_state_update()
{
  // check if both pose and twist are received
  if (!vks_msg_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Waiting for vehicle kinematic state message.");
    return;
  }

  // calculate the global and frenet pose
  Pose2D global_pose {
    vks_msg_->pose.pose.position.x,
    vks_msg_->pose.pose.position.y,
    vks_msg_->car_yaw
  };

  // fill the body velocity
  BodyVelocity2D v_body {
    vks_msg_->speed_mps,
    0.0,
    vks_msg_->velocity.twist.angular.z
  };

  // update the vehicle state message
  const auto now = this->now();
  vehicle_state_msg_->header.stamp = now;
  update_vehicle_state_msg(v_body, global_pose);

  // publish tf
  if (config_->publish_tf) {
    // build the map to cg transform
    tf2::Transform map_to_cg;
    map_to_cg.setOrigin(tf2::Vector3(global_pose.position.x, global_pose.position.y, 0.0));
    map_to_cg.setRotation(utils::TransformHelper::quaternion_from_heading(global_pose.yaw));
    // find the map to baselink transform
    map_to_baselink_msg_->transform = tf2::toMsg(map_to_cg * cg_to_baselink_);
    map_to_baselink_msg_->header.stamp = now;
    // publish the transforms
    tf_helper_.send_transform(*map_to_baselink_msg_);
  }

  // publish odom
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = global_pose.position.x;
  odom.pose.pose.position.y = global_pose.position.y;
  odom.pose.pose.orientation =
    tf2::toMsg(utils::TransformHelper::quaternion_from_heading(global_pose.yaw));
  odom.twist.twist.linear.x = vehicle_state_msg_->v.v_long;
  odom.twist.twist.linear.y = vehicle_state_msg_->v.v_tran;
  odom.twist.twist.angular.z = vehicle_state_msg_->w.w_psi;
  vehicle_odom_pub_->publish(odom);

  // publish the updated state
  vehicle_state_pub_->publish(*vehicle_state_msg_);
}
}  // namespace art_mpc_interface
}  // namespace interface
}  // namespace lmpc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<lmpc::interface::art_mpc_interface::ARTMPCInterfaceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
