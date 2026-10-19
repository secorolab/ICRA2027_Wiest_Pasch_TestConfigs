# Copyright (C) 2025 Frederik Pasch
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions
# and limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    config_dir = os.path.dirname(os.path.abspath(__file__))

    return LaunchDescription([

        DeclareLaunchArgument(
            'laserscan_random_drop_percentage',
            default_value='0.0',
            description='Percentage of random drops in LaserScan'),

        DeclareLaunchArgument(
            'laserscan_gaussian_noise_std_deviation',
            default_value='0.0',
            description='Standard deviation of Gaussian noise in LaserScan'),

        DeclareLaunchArgument(
            'headless',
            default_value='True',
            description='Whether to execute gzclient'),

        # Declared and FORWARDED, matching files/nav2_gazebo.launch.py, which is the sibling that
        # already got this right. It used to be hardcoded to the checked-in file, which silently
        # discarded every parameter the scenario mutates: a scenario writes its overrides to a copy
        # at /out/nav2_params.yaml (run_process + set_yaml_value) and then has no way to make nav2
        # read it, so the Gazebo half ran the stock stack while the roqsim half -- which passes
        # params_file:=/out/nav2_params.yaml explicitly -- ran the varied one. Two halves whose whole
        # claim is "the same stack behind a different simulator" were not running the same stack the
        # moment a relation turned a nav2 knob.
        #
        # The default keeps the previous behaviour for a caller that passes nothing (scenario.osc),
        # so this is additive: only a caller that asks for /out gets /out.
        DeclareLaunchArgument(
            'params_file',
            default_value='/config/files/nav2_params.yaml',
            description='nav2 parameters. Pass the scenario\'s mutated copy '
                        '(/out/nav2_params.yaml) to have its overrides take effect.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/config/files/nav2/tb4_simulation_launch.py']),
            launch_arguments={
                'use_simulator': 'True',
                'params_file': LaunchConfiguration('params_file'),
            }.items()
        ),

        Node(
            package="gazebo_tf_publisher",
            name="gazebo_tf_publisher",
            executable="gazebo_tf_publisher_node",
            parameters=[
                {"gz_pose_topic": "/world/default/dynamic_pose/info"},
                {"base_frame_id": "base_link"},
            ],
        ),

        Node(
            package='message_modification',
            executable='laserscan_modification',
            name='laserscan_modification',
            output='screen',
            remappings=[
                ('/in', '/scan_sim'),
                ('/out', '/scan')
            ],
            parameters=[
                {'random_drop_percentage': LaunchConfiguration('laserscan_random_drop_percentage'),
                 'gaussian_noise_std_deviation': LaunchConfiguration('laserscan_gaussian_noise_std_deviation')}
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([os.path.dirname(os.path.abspath(__file__)), 'monitor_cam_launch.py'])]),
        ),

        ExecuteProcess(
            cmd=['python3', f'{config_dir}/collision_monitor.py'],
            name='collision_monitor',
            output='screen',
        ),

    ])
