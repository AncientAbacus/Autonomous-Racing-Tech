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

from launch import LaunchDescription
from launch_ros.actions import Node
from base_common import get_share_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    input_ttl = get_share_file(
        "race_metadata", "ttls", "LVMS_SVL_ENU_TTL_CSV", "LVMS_SVL_ENU_TTL_LEFT_2.csv"
    )
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="bank_angle_collector",
                executable="bank_angle_collector_node",
                name="bank_angle_collector_node",
                output="screen",
                parameters=[{"input_ttl": input_ttl, "output_ttl": "ttl.csv"}],
                emulate_tty=True,
            ),
        ]
    )
