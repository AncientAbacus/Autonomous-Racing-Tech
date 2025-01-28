// Copyright 2022 Siddharth Saha
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

#include <string>

#include "udp_telemetry/telemetry.hpp"
#include "rclcpp/serialization.hpp"

namespace race
{
namespace telemetry
{
UdpTelemetry::UdpTelemetry(const rclcpp::NodeOptions & options)
: Node("udp_telemetry_node", options), send_telemetry_socket_(io_service_main_),
  recv_basestation_socket_(io_service_main_)
{
  // setup QOS to be best effort
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();

  sub_telemetry_ = this->create_subscription<race_msgs::msg::VehicleTelemetry>(
    "telemetry",
    qos,
    std::bind(&UdpTelemetry::telemetry_callback, this, std::placeholders::_1)
  );

  pub_joystick_command_ = this->create_publisher<race_msgs::msg::VehicleManualControlCommand>(
    "control_command",
    qos
  );
  this->declare_parameter("dt", 0.01);
  auto dt = this->get_parameter("dt").as_double();
  tx_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(dt), [this] {
      tx_timer_callback();
    });
  rx_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(0.01), [this] {
      rx_timer_callback();
    });

  // setup UDP interfaces
  this->declare_parameter<std::string>("send_telemetry_ip", "10.42.7.1");
  send_telemetry_ip_ = this->get_parameter("send_telemetry_ip").as_string();
  this->declare_parameter<int>("send_telemetry_port", 23431);
  send_telemetry_port_ = this->get_parameter("send_telemetry_port").as_int();
  send_telemetry_socket_.open(boost::asio::ip::udp::v4());
  send_telemetry_endpoint_ =
    boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(
      send_telemetry_ip_), send_telemetry_port_);
  RCLCPP_INFO(
    this->get_logger(), "Established connection to send telemetry to : %s:%u",
    send_telemetry_ip_.c_str(), send_telemetry_port_);
  this->declare_parameter<std::string>("recv_basestation_ip", "10.42.7.200");
  recv_basestation_ip_ = this->get_parameter("recv_basestation_ip").as_string();
  this->declare_parameter<int>("recv_basestation_port", 23531);
  recv_basestation_port_ = this->get_parameter("recv_basestation_port").as_int();
  recv_basestation_socket_.open(boost::asio::ip::udp::v4());
  recv_basestation_endpoint_ =
    boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(
      recv_basestation_ip_), recv_basestation_port_);
  recv_basestation_socket_.bind(recv_basestation_endpoint_);
  recv_basestation_socket_.non_blocking(true);
  send_telemetry_endpoint_ =
    boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(
      send_telemetry_ip_), send_telemetry_port_);
  RCLCPP_INFO(
    this->get_logger(), "Established connection to receive basestation commands on : %s:%u",
    recv_basestation_ip_.c_str(), recv_basestation_port_);
}

void UdpTelemetry::telemetry_callback(const race_msgs::msg::VehicleTelemetry::ConstSharedPtr msg)
{
  telemetry_msg_ = *msg;
}

void UdpTelemetry::tx_timer_callback()
{
  static rclcpp::Serialization<race_msgs::msg::VehicleTelemetry> serializer;
  rclcpp::SerializedMessage serialized_msg;
  try {
    rclcpp::SerializedMessage serialized_msg_tmp;
    serializer.serialize_message(&telemetry_msg_, &serialized_msg_tmp);
    serialized_msg = serialized_msg_tmp;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create serialized message: %s", e.what());
    return;
  }
  auto buffer = serialized_msg.get_rcl_serialized_message().buffer;
  auto buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
  boost::system::error_code err;
  send_telemetry_socket_.send_to(
    boost::asio::buffer(
      buffer,
      buffer_length), send_telemetry_endpoint_, 0,
    err);
}

void UdpTelemetry::rx_timer_callback()
{
  static rclcpp::Serialization<race_msgs::msg::VehicleManualControlCommand> serializer;
  static auto last_js_time = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  auto buffer_size = recv_basestation_socket_.available();
  if (!buffer_size) {
    return;
  }
  uint8_t * buffer = new uint8_t[buffer_size];
  boost::system::error_code err;
  recv_basestation_socket_.receive_from(
    boost::asio::buffer(
      buffer,
      buffer_size), recv_basestation_endpoint_, 0,
    err);
  rclcpp::SerializedMessage serialized_msg(rcl_serialized_message_t{buffer, buffer_size,
      buffer_size, rcutils_get_default_allocator()});

  auto joy_msg = race_msgs::msg::VehicleManualControlCommand();
  try {
    serializer.deserialize_message(&serialized_msg, &joy_msg);
  } catch (const std::exception & e) {
    joy_msg = race_msgs::msg::VehicleManualControlCommand();
    RCLCPP_ERROR(get_logger(), e.what());
  }
  if (last_js_time.get_clock_type() != RCL_CLOCK_UNINITIALIZED) {
    try {
      if (rclcpp::Time(joy_msg.header.stamp).seconds() >
        last_js_time.seconds())
      {
        pub_joystick_command_->publish(joy_msg);
        last_js_time = joy_msg.header.stamp;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), e.what());
    }
  } else {
    last_js_time = joy_msg.header.stamp;
  }
}
}  // namespace telemetry
}  // namespace race

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(race::telemetry::UdpTelemetry)
