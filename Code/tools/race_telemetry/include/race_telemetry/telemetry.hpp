// Copyright 2021 AI Racing Tech

#ifndef RACE_TELEMETRY__TELEMETRY_HPP_
#define RACE_TELEMETRY__TELEMETRY_HPP_

#include <string>
#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "race_msgs/msg/vehicle_telemetry.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/ride_height_report.hpp"
#include "race_msgs/msg/push2_pass_report.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/path.hpp"

#include "base_common/pubsub.hpp"
#include "ttl_tree.hpp"

#include "transform_helper/transform_helper.hpp"
#include "race_telemetry/visualizer.hpp"

using pubsub::MsgSubscriber;

namespace race
{
class Telemetry : public rclcpp::Node
{
public:
  explicit Telemetry(const rclcpp::NodeOptions & options);

private:
  void step();

  template<class MsgT>
  void copy_to(
    typename std::unique_ptr<MsgSubscriber<MsgT>> & subscriber,
    MsgT & dst)
  {
    auto msg = subscriber->last_received_msg();
    if (msg) {
      dst = *msg;
    }
  }

  MsgSubscriber<race_msgs::msg::RvcTelemetry>::UniquePtr rvc_tlm_;
  MsgSubscriber<race_msgs::msg::RdeTelemetry>::UniquePtr rde_tlm_;
  MsgSubscriber<race_msgs::msg::RppTelemetry>::UniquePtr rpp_tlm_;
  MsgSubscriber<race_msgs::msg::VehicleCommand>::UniquePtr vehicle_command_;
  MsgSubscriber<race_msgs::msg::VehicleControlCommand>::UniquePtr control_command_;
  MsgSubscriber<race_msgs::msg::VehicleStatus>::UniquePtr vehicle_status_;
  MsgSubscriber<race_msgs::msg::TireTempReport>::UniquePtr tire_temp_;
  MsgSubscriber<race_msgs::msg::TirePressureReport>::UniquePtr lf_tire_pressure_;
  MsgSubscriber<race_msgs::msg::TirePressureReport>::UniquePtr lr_tire_pressure_;
  MsgSubscriber<race_msgs::msg::TirePressureReport>::UniquePtr rf_tire_pressure_;
  MsgSubscriber<race_msgs::msg::TirePressureReport>::UniquePtr rr_tire_pressure_;
  MsgSubscriber<race_msgs::msg::WheelSpeedReport>::UniquePtr wheel_speed_report_;
  MsgSubscriber<race_msgs::msg::WheelPotentiometerReport>::UniquePtr wheel_potentiometer_report_;
  MsgSubscriber<race_msgs::msg::WheelStrainGaugeReport>::UniquePtr wheel_strain_gauge_report_;
  MsgSubscriber<race_msgs::msg::EngineReport>::UniquePtr engine_report_;
  MsgSubscriber<race_msgs::msg::EnginePressuresReport>::UniquePtr engine_pressures_report_;
  MsgSubscriber<race_msgs::msg::RideHeightReport>::UniquePtr ride_height_report_;
  MsgSubscriber<race_msgs::msg::MiscReport>::UniquePtr misc_report_;
  MsgSubscriber<race_msgs::msg::FaultReport>::UniquePtr fault_report_;
  MsgSubscriber<gps_msgs::msg::GPSFix>::UniquePtr gps_fix_;
  MsgSubscriber<sensor_msgs::msg::Imu>::UniquePtr imu_;
  MsgSubscriber<autoware_auto_perception_msgs::msg::TrackedObjects>::UniquePtr tracked_objects_;
  MsgSubscriber<race_msgs::msg::VehicleKinematicState>::UniquePtr vks_;
  MsgSubscriber<race_msgs::msg::Push2PassReport>::UniquePtr p2p_report_;
  // MsgSubscriber<diagnostic_msgs::msg::DiagnosticArray>::UniquePtr diagnostics_;

  MsgSubscriber<race_msgs::msg::SteeringReport>::UniquePtr steering_report_;
  MsgSubscriber<race_msgs::msg::BrakeReport>::UniquePtr brake_report_;
  MsgSubscriber<race_msgs::msg::ThrottleReport>::UniquePtr throttle_report_;
  MsgSubscriber<race_msgs::msg::RaceControlRestOfField>::UniquePtr rof_report_;
  MsgSubscriber<race_msgs::msg::SlipAngleReport>::UniquePtr slip_angle_report_;
  MsgSubscriber<race_msgs::msg::TargetTrajectoryCommand>::UniquePtr sub_ttc_;

  MsgSubscriber<nav_msgs::msg::Path>::UniquePtr sub_ttl_;
  MsgSubscriber<nav_msgs::msg::Path>::UniquePtr sub_rpp_;

  rclcpp::Publisher<race_msgs::msg::VehicleTelemetry>::SharedPtr pub_vehicle_telemetry_;
  rclcpp::Publisher<race_msgs::msg::RdeTelemetry>::SharedPtr pub_rde_tlm_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vis_path_;
  rclcpp::Time prev_time;
  race_msgs::msg::VehicleTelemetry vehicle_telemetry_msg_;
  race_msgs::msg::RdeTelemetry rde_msg_;
  sensor_msgs::msg::Image image_msg_;
  cv_bridge::CvImagePtr cv_ptr_;
  rclcpp::Time start_time;

  double dt_{};
  rclcpp::TimerBase::SharedPtr step_timer_;

  uint8_t vehicle_number_ {};
  uint8_t version_number_ {1};
  uint8_t next_sequence_number_ {};
};

}  // namespace race

#endif  // RACE_TELEMETRY__TELEMETRY_HPP_
