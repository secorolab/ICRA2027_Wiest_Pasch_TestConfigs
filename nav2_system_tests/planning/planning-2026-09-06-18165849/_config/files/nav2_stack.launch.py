# Copyright (C) 2026 Frederik Pasch
# SPDX-License-Identifier: Apache-2.0

"""The navigation stack under test, with no simulator in it — shared by both halves of the pair.

`hexagon_gazebo.vast` and `hexagon_roqsim.vast` compare two simulators behind one nav2. That
claim only holds if both really run the same nav2, so the stack lives in ONE file that both launch
rather than in two that have to be kept in step by hand. What is left over is per-simulator and
small: the Gazebo half wraps this in ``files/nav2_launch.py``, which adds the gz robot spawn, the
gz-pose ground-truth publisher and the gz contact aggregator; the roqsim half launches this file
directly, because its world supplies all three (``spawn_robot``, ``ground_truth_pose``,
``contact_monitor``).

Three nodes, and each is backend-neutral by construction:

* ``robot_state_publisher`` over ``nav2_minimal_tb4_description``'s ``turtlebot4.urdf.xacro`` — the
  same description Gazebo spawns from, so both simulators publish an identical TF tree below
  ``base_link``.
* ``message_modification/laserscan_modification`` rewriting ``/scan_sim`` -> ``/scan``. It is in the
  chain on BOTH halves even at zero noise, so nav2 consumes its scan from the same publisher either
  way and the dataset's ``mt-noise-*`` configurations keep working through one code path. Gazebo's
  bridge names the raw scan ``scan_sim`` (``files/nav2/tb4_bridge.yaml``); roqsim's world does the
  same with ``lidar: {topics: {scan: /scan_sim}}``.
* ``nav2_bringup/bringup_launch.py`` — the twelve lifecycle nodes, the composed ``nav2_container``
  and the localization/navigation lifecycle-manager split.

The bringup defaults here are NOT nav2's own: they are the values
``files/nav2/tb4_simulation_launch.py`` passes, ``use_composition: True`` in particular. Taking this
file off that one's path would silently switch the campaign from a composed container to twelve
separate processes, which is a different system under test — so they are pinned rather than
inherited.

Started with an EMPTY package name and an ABSOLUTE path (``ros_launch('', '/config/files/...')``).
With a package name a remote ``ros_launch`` resolves against the *server's* ament index, and with a
relative path against the client's scenario directory evaluated on the server's filesystem.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# nav2's multi-robot convention, and what bringup_launch.py's own nodes use.
_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_sdf = LaunchConfiguration('robot_sdf')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_namespace', default_value='false'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='the campaign navigates a known map; SLAM is not part of this experiment'),
        DeclareLaunchArgument(
            'map',
            description='occupancy map yaml. Pinned by the campaign to the configuration\'s '
                        'environment, so both simulators localize against the same grid'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            description='nav2 parameters. Pinned by the campaign to /config/files/nav2_params.yaml '
                        'so both simulators run byte-identical nav2 configuration'),
        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='activate the nav2 lifecycle nodes at launch. FALSE by default so starting '
                        'this file by hand does not bring nav2 up against a simulator that is not '
                        'publishing yet; each scenario passes what its own gate makes safe'),
        # From tb4_simulation_launch.py, not from bringup_launch.py (which defaults to False).
        # Composition changes the process topology of the system under test, so it is stated.
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument(
            'use_rviz', default_value='False',
            description='off for a campaign on either backend; the dataset\'s local runs turn it on'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'),
            description='the TurtleBot 4 description. nav2_minimal_tb4_description and not the '
                        'fuller turtlebot4_description: a different package is a different TF tree'),
        DeclareLaunchArgument(
            'laserscan_random_drop_percentage', default_value='0.0',
            description='percentage of returns dropped from each scan'),
        DeclareLaunchArgument(
            'laserscan_gaussian_noise_std_deviation', default_value='0.0',
            description='standard deviation of the Gaussian range noise added to each scan'),
        DeclareLaunchArgument(
            'scan_in', default_value='/scan_sim',
            description='the simulator\'s raw scan, before noise is applied'),
        DeclareLaunchArgument(
            'scan_out', default_value='/scan',
            description='what nav2 and its collision_monitor consume'),
        DeclareLaunchArgument('planner_case', default_value='none'),

        # base_link and everything below it (base_footprint, wheels, rplidar_link) from the model,
        # consuming the simulator's /joint_states. Both worlds keep their bridge/gz from publishing
        # these same static transforms: two publishers for one static transform is a TF conflict,
        # not redundancy, and nav2's collision_monitor then cannot transform its scan and — sitting
        # in the cmd_vel path — fails closed at zero velocity.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', robot_sdf]),
            }],
            remappings=_REMAPPINGS,
        ),

        # The sensor-degradation channel, in the chain on both backends. At the default 0.0/0.0 it
        # is a pass-through, which is what mt-hexagon-normal wants; the mt-noise-* configurations
        # sweep these two arguments and need no other change.
        Node(
            package='message_modification',
            executable='laserscan_modification',
            name='laserscan_modification',
            namespace=namespace,
            output='screen',
            remappings=[('/in', LaunchConfiguration('scan_in')),
                        ('/out', LaunchConfiguration('scan_out'))],
            parameters=[{
                'use_sim_time': use_sim_time,
                'random_drop_percentage': LaunchConfiguration(
                    'laserscan_random_drop_percentage'),
                'gaussian_noise_std_deviation': LaunchConfiguration(
                    'laserscan_gaussian_noise_std_deviation'),
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'slam': slam,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
            }.items(),
        ),

        ExecuteProcess(
            cmd=['python3', '/config/files/planner_path_valid_helper.py',
                 '--case', LaunchConfiguration('planner_case')],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'use_sim_time': use_sim_time,
                'rviz_config': LaunchConfiguration('rviz_config_file'),
            }.items(),
        ),
    ])
