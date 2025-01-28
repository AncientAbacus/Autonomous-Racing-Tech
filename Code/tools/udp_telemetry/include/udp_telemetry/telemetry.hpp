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

#ifndef UDP_TELEMETRY__TELEMETRY_HPP_
#define UDP_TELEMETRY__TELEMETRY_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_telemetry.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"

namespace race
{
namespace telemetry
{
class UdpTelemetry : public rclcpp::Node
{
public:
  explicit UdpTelemetry(const rclcpp::NodeOptions & options);

private:
  void tx_timer_callback();
  void rx_timer_callback();
  void telemetry_callback(const race_msgs::msg::VehicleTelemetry::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::Subscription<race_msgs::msg::VehicleTelemetry>::SharedPtr sub_telemetry_;
  rclcpp::Publisher<race_msgs::msg::VehicleManualControlCommand>::SharedPtr pub_joystick_command_;

  race_msgs::msg::VehicleTelemetry telemetry_msg_;

  boost::asio::io_service io_service_main_;
  boost::asio::ip::udp::socket send_telemetry_socket_;
  boost::asio::ip::udp::socket recv_basestation_socket_;
  boost::asio::ip::udp::endpoint send_telemetry_endpoint_;
  boost::asio::ip::udp::endpoint recv_basestation_endpoint_;
  std::string send_telemetry_ip_;
  unsigned int send_telemetry_port_;
  std::string recv_basestation_ip_;
  unsigned int recv_basestation_port_;
};
}  // namespace telemetry
}  // namespace race

#endif  // UDP_TELEMETRY__TELEMETRY_HPP_
