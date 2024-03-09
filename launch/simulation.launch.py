# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    px4_offboard_path = get_package_share_directory('px4_offboard')
    grad_traj_path = get_package_share_directory('grad_traj_optimization')
    use_sim_time = True
    # Bridge
    # 对应表: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/gz_imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/model/x500_mono_cam_0/servo_lidar@std_msgs/msg/Float64@gz.msgs.Double',
                   '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                   '/servo_lidar_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
        output='screen'
    )
    # tf lidar2baselink
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar2baselink',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'x500_mono_cam_0/my_lidar/base_link/gpu_lidar']
        )
    offboardNode = Node(
        package="px4_offboard",
        executable="offboard",
    )
    rotationLidarNode = Node(
        package="px4_offboard",
        parameters=[{'use_sim_time': use_sim_time}],
        executable="rotationLidar_tool",
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["-d", px4_offboard_path+"/launch/simulation.rviz"]
    )
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(grad_traj_path+'/launch/text_input.launch.py')
    )
    return LaunchDescription([
        bridge,
        rviz2,
        offboardNode,
        rotationLidarNode
        # included_launch,
        # tf_lidar
    ])