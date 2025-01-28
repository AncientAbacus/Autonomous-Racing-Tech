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

#include "ghost_car/ghost_car_node.hpp"

#include <string>
#include <vector>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "ghost_car/controller_plugin.hpp"

namespace race::ghost_car
{
GhostCar::GhostCar(const rclcpp::NodeOptions & options)
: Node("ghost_car_node", options), controller_loader_("ghost_car",
    "race::ghost_car::GhostCarController")
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // Loading vehicle model configurations
  m_model_ = std::make_shared<race::vehicle_model::VehicleModel>(
    race::vehicle_model::VehicleModelConfig::SharedPtr());
  race::vehicle_model::load_parameters(this, *m_model_);

  // setting up controller
  std::string controller_type;
  // need to use environs for sure
  this->declare_parameter<std::string>("controller_type");
  controller_type = this->get_parameter("controller_type").as_string();

  std::string controller_name = "race::ghost_car::" + controller_type;

  try {
    controller_ = controller_loader_.createSharedInstance(controller_name);
    controller_->initialize(this, m_model_);
    controller_->configure();
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create controller plugin. Exception: %s", ex.what());
    rclcpp::shutdown();
  }

  pub_ghost_car_ = create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>(
    "tracked_objects", rclcpp::SensorDataQoS());

  this->declare_parameter<double>("dt");

  dt_ = get_parameter("dt").as_double();

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&GhostCar::parametersCallback, this, std::placeholders::_1));
  step_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(dt_), [this] {
      step();
    });
  latest_time_ = this->now();

  ghost_car_count_ = get_parameter("ghost_car_count").as_int();
  RCLCPP_WARN(get_logger(), "Ghost car count: %d", ghost_car_count_);
  ghost_car_num_to_publish_ = declare_parameter<int>("ghost_car_num_to_publish", ghost_car_count_);
}

rcl_interfaces::msg::SetParametersResult GhostCar::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const auto & param : parameters) {
    if (param.get_name() == "ghost_car_num_to_publish" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      if (param.as_int() <= ghost_car_count_) {
        ghost_car_num_to_publish_ = param.as_int();
        result.successful = true;
        break;
      }
    }
    result.successful = controller_->update_param(param);
    if (!result.successful) {
      break;
    }
  }
  result.reason = result.successful ? "success" : "failure";
  return result;
}

void GhostCar::step()
{
  double speed;
  double yaw_angle;
  rclcpp::Time current_time;
  tf2::Quaternion orientation;
  autoware_auto_perception_msgs::msg::TrackedObjects tracked_objects;
  tracked_objects.header.frame_id = "map";
  tracked_objects.header.stamp = this->now();

  for (int i = 0; i < ghost_car_num_to_publish_; i++) {
    controller_->updatePositionAndSpeed(positions_[i], speed, yaw_angle, i);

    // set orientation angle
    orientation.setRPY(0, 0, yaw_angle);

    // Create and publish the ghost car
    autoware_auto_perception_msgs::msg::ObjectClassification ghost_car_classification;
    ghost_car_classification.probability = 1.0;
    ghost_car_classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
    autoware_auto_perception_msgs::msg::TrackedObject ghost_car;
    ghost_car.shape.type = ghost_car.shape.BOUNDING_BOX;
    ghost_car.shape.dimensions.x = 8.0;
    ghost_car.shape.dimensions.y = 4.0;
    ghost_car.shape.dimensions.z = 2.0;
    ghost_car.kinematics.pose_with_covariance.pose.position.x = positions_[i].x;
    ghost_car.kinematics.pose_with_covariance.pose.position.y = positions_[i].y;
    ghost_car.kinematics.pose_with_covariance.pose.orientation = tf2::toMsg(orientation);
    ghost_car.existence_probability = 1.0;
    ghost_car.kinematics.twist_with_covariance.twist.linear.x = speed;
    ghost_car.classification =
      std::vector<autoware_auto_perception_msgs::msg::ObjectClassification>(
      {ghost_car_classification});
    ghost_car.object_id.uuid[0] = i;

    // Publish the ghost car
    tracked_objects.objects.push_back(ghost_car);

    // Creating a fram for the ghost car
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = current_time;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "ghost_car_frame_" + std::to_string(i);
    transformStamped.transform.translation.x = positions_[i].x;
    transformStamped.transform.translation.y = positions_[i].y;
    transformStamped.transform.translation.z = 0.0;  // Assuming z is 0

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_angle);  // Set orientation
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transformStamped);
  }

  pub_ghost_car_->publish(tracked_objects);
}

}  // namespace race::ghost_car

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::ghost_car::GhostCar>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
