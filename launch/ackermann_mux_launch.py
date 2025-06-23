#!/usr/bin/env python3
# Copyright 2020 Gaitech Korea Co., Ltd.
# Copyright 2024 Elouarn de KERROS
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

# Author: Brighten Lee
# Author: Elouarn de KERROS

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_locks = os.path.join(get_package_share_directory('ackermann_mux'),
                                        'config', 'ackermann_mux_locks.yaml')
    default_config_topics = os.path.join(get_package_share_directory('ackermann_mux'),
                                         'config', 'ackermann_mux_topics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'cmd_out',
            default_value='ackermann_cmd',
            description='ackermann cmd output topic'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'),
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            output='screen',
            remappings={('/ackermann_cmd', LaunchConfiguration('cmd_out'))},
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_locks'),
                LaunchConfiguration('config_topics')]
        ),
        # Node(
        #     package='ackermann_mux',
        #     executable='ackermann_marker',
        #     output='screen',
        #     remappings={('/ackermann_cmd', LaunchConfiguration('cmd_out'))},
        #     parameters=[{
        #         'use_sim_time': LaunchConfiguration('use_sim_time'),
        #         'frame_id': 'base_footprint',
        #         'scale': 1.0,
        #         'vertical_position': 2.0}])
    ])
