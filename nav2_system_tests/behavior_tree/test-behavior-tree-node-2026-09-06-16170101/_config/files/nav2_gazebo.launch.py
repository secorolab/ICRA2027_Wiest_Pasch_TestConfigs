# Copyright (C) 2026 Frederik Pasch
# SPDX-License-Identifier: Apache-2.0

"""The Gazebo half's `sut` container: the shared nav2 stack, plus the three things only gz needs.

``files/nav2_stack.launch.py`` is the system under test and is launched unchanged on both
simulators. This file adds what Gazebo has no other source for, and what the roqsim world supplies
from plugins instead:

===========================  =======================================  ============================
what                         here (Gazebo)                            roqsim's equivalent
===========================  =======================================  ============================
put the robot in the world   ``files/nav2/spawn_tb4.launch.py``        ``spawn_robot`` in the world
ground-truth pose frame      ``gazebo_tf_publisher``                   ``ground_truth_pose`` plugin
collision signal             ``files/collision_monitor.py``            ``contact_monitor`` plugin
a picture of the trial       ``files/monitor_cam_launch.py``           the run capture ``scene3d``
                             (a camera in the world)                   replays
===========================  =======================================  ============================

The first three produce the SAME observable on both sides — a robot at the configuration's start
pose, a ``nav2_turtlebot4_base_link_gt`` TF frame, and ``std_msgs/Bool`` on ``/collision`` — which
is what lets one measurement half of the scenario, one postprocessing block and one set of notebooks
serve both simulators.

The fourth is the one thing the two halves do differently ON PURPOSE, and it is not part of the
measurement. roqsim records a capture the ``scene3d`` panel replays; Gazebo can produce no such
thing, so it gets a camera IN the world instead and its Run view plays the video. Nothing the trial
is measured from touches either one — what changes is only whether a person opening the run can see
what happened, which without this they could not on this half.

It is a sibling of ``files/nav2_launch.py`` rather than a rewrite of it: that file belongs to
``metamorphic_new.vast``, whose 34 configurations still run through
``files/nav2/tb4_simulation_launch.py`` (which bundles the gz spawn into the nav2 bring-up). This
pair needs the spawn separated from the stack, so it takes the pieces apart here and leaves the v1
sweep's path untouched.

Two values are load-bearing:

* ``robot_name`` is ``nav2_turtlebot4``. ``gazebo_tf_publisher`` names the frame
  ``<gz model name>_<base_frame_id>_gt``, so this is what makes the ground-truth frame
  ``nav2_turtlebot4_base_link_gt`` — the name ``metamorphic_new.vast``'s postprocessing and the
  analysis notebooks already read, and the name the roqsim world reproduces.
* ``gz_pose_topic`` embeds the gz WORLD name, and ``files/world.sdf.xacro`` declares
  ``<world name='default'>``. A wrong world name is not an error: the node subscribes happily and
  never receives a message, which looks exactly like the missing-frame bug it exists to prevent.

``use_sim_time`` is set on ``gazebo_tf_publisher``, which ``files/nav2_launch.py`` does not do. The
ground-truth frame is the trajectory the comparison is measured from; stamped in wall time while
every other topic in the bag is on the simulator's clock, it cannot be aligned with them.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The gz world whose pose feed carries the robot: files/world.sdf.xacro's <world name='default'>.
_WORLD = 'default'
# Must match the name the roqsim world gives its ground-truth frame, so the analysis needs no
# per-backend special case.
_ROBOT_NAME = 'nav2_turtlebot4'


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))

    return LaunchDescription([
        # Forwarded to the shared stack, which declares them with descriptions. Repeating them here
        # is what makes them settable from the scenario's key_value list: an include shares the
        # parent's launch context, but only declared-or-passed arguments arrive with a value.
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('laserscan_random_drop_percentage', default_value='0.0'),
        DeclareLaunchArgument('laserscan_gaussian_noise_std_deviation', default_value='0.0'),
        # Where gz puts the robot. The roqsim half applies the same pose with set_entity_state,
        # because a simulator in its own container has no command line to hang it on.
        DeclareLaunchArgument('x_pose', default_value='0.00'),
        DeclareLaunchArgument('y_pose', default_value='0.00'),
        DeclareLaunchArgument('yaw', default_value='0.00'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'nav2_stack.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'laserscan_random_drop_percentage': LaunchConfiguration(
                    'laserscan_random_drop_percentage'),
                'laserscan_gaussian_noise_std_deviation': LaunchConfiguration(
                    'laserscan_gaussian_noise_std_deviation'),
            }.items(),
        ),

        # gz spawns from the /robot_description the stack's robot_state_publisher publishes, so this
        # has to follow it in the same launch — which is also why it cannot live in the shared file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'nav2', 'spawn_tb4.launch.py')),
            launch_arguments={
                'use_simulator': 'True',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_name': _ROBOT_NAME,
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'yaw': LaunchConfiguration('yaw'),
            }.items(),
        ),

        # Ground truth: gz's own pose feed republished as map -> nav2_turtlebot4_base_link_gt.
        # Without it the run records only base_link — the AMCL *estimate* — so a "ground truth"
        # comparison would silently be comparing believed poses.
        Node(
            package='gazebo_tf_publisher',
            executable='gazebo_tf_publisher_node',
            name='gazebo_tf_publisher',
            output='screen',
            parameters=[{
                'gz_pose_topic': f'/world/{_WORLD}/dynamic_pose/info',
                'base_frame_id': 'base_link',
                'use_sim_time': True,
            }],
        ),

        # The failure oracle: subscribes to every /contact/* the gz bridge exposes (the wall mesh
        # here; obstacle models in the configurations that spawn them) and latches std_msgs/Bool
        # True on /collision. roqsim's contact_monitor publishes the same topic and type from the
        # robot's own contacts, so the scenario check is identical on both.
        ExecuteProcess(
            cmd=['python3', os.path.join(here, 'collision_monitor.py')],
            name='collision_monitor',
            output='screen',
        ),

        # The monitor camera: a static top-down camera spawned into the gz world, framed on the
        # map, publishing /static_camera/image/compressed at 1 Hz. `rosbags_to_webm` turns the
        # recorded frames into the .webm the Run view's `camera` panel plays.
        #
        # This is what this half has INSTEAD of a 3D scene, not in addition to one. roqsim
        # writes a run capture and the `scene3d` panel replays it; Gazebo writes no capture and
        # has no scene exporter, so without this its Run view has no picture of the trial at all.
        # `monitor_cam_launch.py` reads the map to place the camera, which is why it needs `map`.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'monitor_cam_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map')}.items(),
        ),
    ])
