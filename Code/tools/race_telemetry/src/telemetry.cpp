// Copyright 2021 AI Racing Tech
//
// Publishes telemetry messages for the vehicle.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "race_telemetry/telemetry.hpp"

using std::chrono::duration;

namespace race
{

Telemetry::Telemetry(const rclcpp::NodeOptions & options)
: rclcpp::Node("telemetry", options)
{
  vehicle_number_ = declare_parameter("vehicle_number", 22);
  version_number_ = declare_parameter("version_number", 3);

  pubsub::subscribe_from(this, rvc_tlm_, "rvc_telemetry");
  pubsub::subscribe_from(this, rde_tlm_, "rde_telemetry");
  pubsub::subscribe_from(this, rpp_tlm_, "rpp_telemetry");
  pubsub::subscribe_from(this, vehicle_command_, "race_vehicle_command");
  pubsub::subscribe_from(this, control_command_, "control_command");
  pubsub::subscribe_from(this, vehicle_status_, "race_vehicle_status");
  pubsub::subscribe_from(this, tire_temp_, "tire_temperature");
  pubsub::subscribe_from(this, lf_tire_pressure_, "race_fl_tire_pressure");
  pubsub::subscribe_from(this, lr_tire_pressure_, "race_fr_tire_pressure");
  pubsub::subscribe_from(this, rf_tire_pressure_, "race_rl_tire_pressure");
  pubsub::subscribe_from(this, rr_tire_pressure_, "race_rr_tire_pressure");
  pubsub::subscribe_from(this, wheel_speed_report_, "race_wheel_speed_report");
  pubsub::subscribe_from(this, wheel_potentiometer_report_, "race_wheel_potentiometer_report");
  pubsub::subscribe_from(this, wheel_strain_gauge_report_, "race_wheel_strain_gauge_report");
  pubsub::subscribe_from(this, engine_report_, "race_engine_report");
  pubsub::subscribe_from(this, engine_pressures_report_, "race_engine_pressures_report");
  pubsub::subscribe_from(this, ride_height_report_, "race_ride_height_report");
  pubsub::subscribe_from(this, gps_fix_, "gps_fix");
  pubsub::subscribe_from(this, imu_, "imu");
  pubsub::subscribe_from(this, misc_report_, "race_misc_report");
  pubsub::subscribe_from(this, fault_report_, "race_fault_report");
  pubsub::subscribe_from(this, tracked_objects_, "tracked_objects");
  pubsub::subscribe_from(this, vks_, "vehicle_kinematic_state");
  pubsub::subscribe_from(this, p2p_report_, "push2pass_report");
  pubsub::subscribe_from(this, steering_report_, "steering_report");
  pubsub::subscribe_from(this, brake_report_, "brake_report");
  pubsub::subscribe_from(this, throttle_report_, "throttle_report");
  pubsub::subscribe_from(this, rof_report_, "race_rof_report");
  pubsub::subscribe_from(this, slip_angle_report_, "slip_angle_report");
  pubsub::subscribe_from(this, sub_ttl_, "ttl");
  pubsub::subscribe_from(this, sub_rpp_, "/rpp/path_command");
  pubsub::subscribe_from(this, sub_ttc_, "/rde/trajectory_command");

  pubsub::publish_to(this, pub_vehicle_telemetry_, "telemetry");

  vehicle_telemetry_msg_ = race_msgs::msg::VehicleTelemetry();
  prev_time = now();
  rde_msg_ = race_msgs::msg::RdeTelemetry();
  dt_ = declare_parameter("dt", 0.01);
  step_timer_ = rclcpp::create_timer(this, get_clock(), duration<float>(dt_), [this] {step();});
}

void Telemetry::step()
{
  auto & msg = vehicle_telemetry_msg_;

  auto diff = now() - prev_time;
  prev_time = now();

  msg.header.stamp = now();
  msg.header.vehicle_number = vehicle_number_;
  msg.header.sequence_number = next_sequence_number_;
  next_sequence_number_ += 1;

  copy_to(rvc_tlm_, msg.rvc_tlm);
  copy_to(rde_tlm_, msg.rde_tlm);
  copy_to(rpp_tlm_, msg.rpp_tlm);
  copy_to(vehicle_command_, msg.vehicle_cmd);
  copy_to(control_command_, msg.control_cmd);
  copy_to(vehicle_status_, msg.vehicle_status);
  copy_to(tire_temp_, msg.tire_temp);
  copy_to(lf_tire_pressure_, msg.lf_tire_pressure_rpt);
  copy_to(lr_tire_pressure_, msg.lr_tire_pressure_rpt);
  copy_to(rf_tire_pressure_, msg.rf_tire_pressure_rpt);
  copy_to(rr_tire_pressure_, msg.rr_tire_pressure_rpt);
  copy_to(wheel_speed_report_, msg.wheel_speed_rpt);
  copy_to(wheel_potentiometer_report_, msg.wheel_potentiometer_rpt);
  copy_to(wheel_strain_gauge_report_, msg.wheel_strain_gauge_rpt);
  copy_to(engine_report_, msg.engine_rpt);
  copy_to(engine_pressures_report_, msg.engine_pressures_rpt);
  copy_to(ride_height_report_, msg.ride_height_rpt);
  copy_to(gps_fix_, msg.gps_fix);
  copy_to(imu_, msg.imu);
  copy_to(misc_report_, msg.do_misc_rpt);
  copy_to(fault_report_, msg.fault_rpt);
  copy_to(p2p_report_, msg.push2pass_rpt);
  // copy_to(tracked_objects_, msg.tracked_objects);
  vision_msgs::msg::Detection3DArray det3d_arr;
  if (tracked_objects_->has_seen_msg()) {
    auto tracked_objects_msg = tracked_objects_->last_received_msg();
    det3d_arr.header = tracked_objects_msg->header;
    for (auto & object : tracked_objects_msg->objects) {
      vision_msgs::msg::Detection3D det_3d;
      det_3d.header = tracked_objects_msg->header;
      det_3d.bbox.center = object.kinematics.pose_with_covariance.pose;
      det_3d.bbox.size.x = 2.0;
      det_3d.bbox.size.y = 4.0;
      det_3d.bbox.size.z = 1.0;
      det3d_arr.detections.push_back(det_3d);
    }
    msg.tracked_objects = det3d_arr;
  }
  copy_to(vks_, msg.vks);
  copy_to(steering_report_, msg.steering_rpt);
  copy_to(brake_report_, msg.brake_rpt);
  copy_to(throttle_report_, msg.throttle_rpt);
  copy_to(rof_report_, msg.rof_rpt);
  copy_to(slip_angle_report_, msg.slip_rpt);
  copy_to(sub_ttc_, msg.target_trajectory_cmd);
  msg.long_error = (msg.rvc_tlm.target_speed - msg.vks.speed_mps) * diff.seconds();
  // TODO(haoru): VKS and autoware tracked objects

  pub_vehicle_telemetry_->publish(msg);
}
}  // namespace race

RCLCPP_COMPONENTS_REGISTER_NODE(race::Telemetry)
