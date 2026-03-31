# Copyright 2026 ROBOTIS AI CO., LTD.
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
#
# ANTBot Ignition Gazebo simulation launch file.
#
# Usage:
#   ros2 launch antbot_gazebo gazebo.launch.py                  # empty world
#   ros2 launch antbot_gazebo gazebo.launch.py world:=depot      # by name
#   ros2 launch antbot_gazebo gazebo.launch.py world:=/full/path/to/world.sdf
#
# World names are resolved via antbot_navigation/maps/worlds.yaml.
#
# Author: Yeeun Hwang

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _resolve_world_path(context, *args, **kwargs):
    """Resolve world name to SDF path using worlds.yaml, then launch Gazebo."""
    world_value = LaunchConfiguration('world').perform(context)

    # If it's already a full path, use it directly
    if os.path.isfile(world_value):
        world_sdf = world_value
    else:
        # Treat as a world name — look up in worlds.yaml
        nav_pkg = get_package_share_directory('antbot_navigation')
        maps_dir = os.path.join(nav_pkg, 'maps')
        worlds_yaml = os.path.join(maps_dir, 'worlds.yaml')

        if os.path.isfile(worlds_yaml):
            with open(worlds_yaml, 'r') as f:
                config = yaml.safe_load(f)
            worlds = config.get('worlds', {})
            if world_value in worlds:
                world_sdf = os.path.join(maps_dir, worlds[world_value]['sdf'])
            else:
                # Fallback: try as filename in maps directory
                world_sdf = os.path.join(maps_dir, world_value + '.sdf')
        else:
            world_sdf = os.path.join(maps_dir, world_value + '.sdf')

        if not os.path.isfile(world_sdf):
            raise FileNotFoundError(
                f"World '{world_value}' not found. Looked for: {world_sdf}")

    return [ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_sdf],
        output='screen')]


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('antbot_gazebo')
    description_pkg = get_package_share_directory('antbot_description')

    # Plugin path based on ROS distro
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    plugin_path = os.path.join('/opt', 'ros', ros_distro, 'lib')

    # ── Launch arguments ──
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World name (from worlds.yaml) or full path to SDF file')

    # ── Environment variables ──
    resource_path = os.path.join(description_pkg, os.pardir)
    existing_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        resource_path + (':' + existing_resource if existing_resource else ''))

    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    set_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        plugin_path + (':' + existing_plugin if existing_plugin else ''))

    # ── URDF processing ──
    urdf_path = os.path.join(gazebo_pkg, 'urdf', 'antbot_sim.xacro')
    robot_description_xml = xacro.process_file(urdf_path).toxml()

    # ── Launch Gazebo (resolved via OpaqueFunction) ──
    ign_gazebo = OpaqueFunction(function=_resolve_world_path)

    # ── Spawn robot ──
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'antbot',
            '-string', robot_description_xml,
            '-x', '0.0', '-y', '0.0', '-z', '0.5',
        ],
        output='screen')

    # ── Robot State Publisher ──
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_xml},
            {'use_sim_time': True},
        ])

    # ── Controller configuration ──
    controller_yaml = os.path.join(
        gazebo_pkg, 'config', 'swerve_controller_gazebo.yaml')

    # ── Wait for gz_ros2_control to load controllers, then activate ──
    # gz_ros2_control plugin auto-loads controllers from YAML.
    # We poll until controllers appear in 'unconfigured' state, then
    # configure and activate them via service calls (no spawner conflict).
    activate_controllers = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "[gazebo.launch] Waiting for controllers to be loaded by gz_ros2_control..." && '
            'until ros2 control list_controllers 2>/dev/null '
            '| grep -q unconfigured; do sleep 1; done && '
            'echo "[gazebo.launch] Controllers loaded, configuring and activating..." && '
            'ros2 service call /controller_manager/configure_controller '
            'controller_manager_msgs/srv/ConfigureController '
            '"{name: joint_state_broadcaster}" && '
            'ros2 service call /controller_manager/switch_controller '
            'controller_manager_msgs/srv/SwitchController '
            '"{activate_controllers: [joint_state_broadcaster], '
            'deactivate_controllers: [], strictness: 1}" && '
            'ros2 service call /controller_manager/configure_controller '
            'controller_manager_msgs/srv/ConfigureController '
            '"{name: antbot_swerve_controller}" && '
            'ros2 service call /controller_manager/switch_controller '
            'controller_manager_msgs/srv/SwitchController '
            '"{activate_controllers: [antbot_swerve_controller], '
            'deactivate_controllers: [], strictness: 1}" && '
            'echo "[gazebo.launch] All controllers active."'
        ],
        output='screen')

    # Start controller activation after robot entity is created in Gazebo
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[activate_controllers]))

    # ── Gazebo - ROS 2 topic bridge ──
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan_0@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            ('/imu', '/imu/data'),
        ],
        output='screen')

    return LaunchDescription([
        # Arguments
        world_arg,
        # Environment
        set_resource_path,
        set_plugin_path,
        # Gazebo
        ign_gazebo,
        spawn_robot,
        robot_state_pub,
        # Controllers (spawn_robot exit → poll hardware → JSB → swerve)
        controllers_after_spawn,
        # Topic bridge
        gz_bridge,
    ])
