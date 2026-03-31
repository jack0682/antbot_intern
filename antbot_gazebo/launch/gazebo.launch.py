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
    """Resolve world name to SDF path, then launch Gazebo."""
    world_value = LaunchConfiguration('world').perform(context)

    if os.path.isfile(world_value):
        world_sdf = world_value
    else:
        gazebo_pkg = get_package_share_directory('antbot_gazebo')
        worlds_yaml = os.path.join(gazebo_pkg, 'config', 'worlds.yaml')
        worlds_dir = os.path.join(gazebo_pkg, 'worlds')

        if os.path.isfile(worlds_yaml):
            with open(worlds_yaml, 'r') as f:
                config = yaml.safe_load(f)
            worlds = config.get('worlds', {})
            if world_value in worlds:
                world_sdf = os.path.join(worlds_dir, worlds[world_value]['sdf'])
            else:
                world_sdf = os.path.join(worlds_dir, world_value + '.sdf')
        else:
            world_sdf = os.path.join(worlds_dir, world_value + '.sdf')

        if not os.path.isfile(world_sdf):
            raise FileNotFoundError(
                f"World '{world_value}' not found. Looked for: {world_sdf}")

    return [ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_sdf],
        output='screen')]


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('antbot_gazebo')
    description_pkg = get_package_share_directory('antbot_description')

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    plugin_path = os.path.join('/opt', 'ros', ros_distro, 'lib')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World name (from worlds.yaml) or full path to SDF file')
    resource_path = os.path.join(description_pkg, os.pardir)
    existing_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        resource_path + (':' + existing_resource if existing_resource else ''))

    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    set_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        plugin_path + (':' + existing_plugin if existing_plugin else ''))

    urdf_path = os.path.join(gazebo_pkg, 'urdf', 'antbot_sim.xacro')
    robot_description_xml = xacro.process_file(urdf_path).toxml()

    ign_gazebo = OpaqueFunction(function=_resolve_world_path)

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'antbot',
            '-string', robot_description_xml,
            '-x', '0.0', '-y', '0.0', '-z', '0.5',
        ],
        output='screen')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_xml},
            {'use_sim_time': True},
        ])

    controller_yaml = os.path.join(
        gazebo_pkg, 'config', 'swerve_controller_gazebo.yaml')

    wait_for_cm = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "[gazebo.launch] Waiting for controller_manager..." && '
            'until ros2 service list 2>/dev/null | grep -q /controller_manager/list_controllers; '
            'do sleep 1; done && '
            'sleep 2 && '
            'echo "[gazebo.launch] controller_manager is ready."'
        ],
        output='screen')

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', controller_yaml,
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    swerve_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'antbot_swerve_controller',
            '--param-file', controller_yaml,
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    wait_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[wait_for_cm]))

    jsb_after_cm = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_cm,
            on_exit=[jsb_spawner]))

    swerve_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[swerve_spawner]))

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
        world_arg,
        set_resource_path,
        set_plugin_path,
        ign_gazebo,
        spawn_robot,
        robot_state_pub,
        wait_after_spawn,
        jsb_after_cm,
        swerve_after_jsb,
        gz_bridge,
    ])
