# Copyright (c) 2018 Intel Corporation
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_toolbox_launch = os.path.join(
        get_package_share_directory('tianracer_slam'),
        'launch', 'slam_toolbox.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_namespace = LaunchConfiguration('use_namespace', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_namespace', default_value='true',
            description='Whether to apply a namespace to the SLAM stack'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
            }.items()),
    ])
