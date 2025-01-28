# Copyright 2022 AI Racing Tech
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from race_msgs.msg import VehicleKinematicState
from trajectory_tools.utils.utils import save_ttl, load_ttl
from trajectory_tools.simulator.model.trajectory import Trajectory
import numpy as np
import tf_transformations


class BankAngleCollector(Node):
    def __init__(self):
        super().__init__("bank_angle_collector_node")
        self.vks_sub = self.create_subscription(
            VehicleKinematicState, "/vehicle/state", self.on_vks, qos_profile_sensor_data
        )
        self.input_ttl_file = (
            self.declare_parameter("input_ttl").get_parameter_value().string_value
        )
        self.output_ttl_file = (
            self.declare_parameter("output_ttl").get_parameter_value().string_value
        )
        self.get_logger().info(f"Loading TTL from {self.input_ttl_file}")
        self.ttl = load_ttl(self.input_ttl_file)
        self.get_logger().info("TTL loaded.")
        self.status_timer = self.create_timer(1.0, self.on_status_timer)

    def on_vks(self, msg: VehicleKinematicState):
        pos = np.array([msg.pose.position.x, msg.pose.position.y])
        o = msg.pose.orientation
        roll, _, _ = tf_transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        closest_ttl_idx = self.ttl.find_closest(pos)
        self.ttl[closest_ttl_idx, Trajectory.BANK] = roll

    def save(self):
        self.get_logger().info(f"Saving TTL to {self.output_ttl_file}")
        save_ttl(self.output_ttl_file, self.ttl)
        self.get_logger().info("Save successful. Node exiting.")

    def on_status_timer(self):
        count = np.sum(self.ttl[:, Trajectory.BANK] != 0.0)
        total = len(self.ttl)
        self.get_logger().info(f"{count}/{total} trajectory points have banking filled.")
        if count == total:
            self.save()
            exit()


def main(args=None):
    rclpy.init(args=args)
    node = BankAngleCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
