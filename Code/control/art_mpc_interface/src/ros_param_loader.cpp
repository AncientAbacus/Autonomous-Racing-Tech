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

#include <lmpc_utils/ros_param_helper.hpp>

#include <string>
#include <memory>
#include <vector>

#include "art_mpc_interface/ros_param_loader.hpp"

namespace lmpc
{
namespace interface
{
namespace art_mpc_interface
{
ARTMPCInterfaceConfig::SharedPtr load_parameters(rclcpp::Node * node)
{
  auto declare_double = [&](const char * name) {
      return lmpc::utils::declare_parameter<double>(node, name);
    };
  // auto declare_string = [&](const char * name) {
  //     return lmpc::utils::declare_parameter<std::string>(node, name);
  //   };
  // auto declare_vec = [&](const char * name) {
  //     return lmpc::utils::declare_parameter<std::vector<double>>(node, name);
  //   };
  // auto declare_int = [&](const char * name) {
  //     return lmpc::utils::declare_parameter<int64_t>(node, name);
  //   };
  auto declare_bool = [&](const char * name) {
      return lmpc::utils::declare_parameter<bool>(node, name);
    };

  return std::make_shared<ARTMPCInterfaceConfig>(
    ARTMPCInterfaceConfig{
          declare_bool("art_mpc_interface.publish_tf"),
          declare_double("art_mpc_interface.min_vx"),
          declare_double("art_mpc_interface.steering_scale")
        }
  );
}
}  // namespace art_mpc_interface
}  // namespace interface
}  // namespace lmpc
