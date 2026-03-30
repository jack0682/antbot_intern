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
# Ignition Gazebo simulation launch for ANTBot.
# Usage: ros2 launch antbot_bringup sim.launch.py
#        ros2 launch antbot_bringup sim.launch.py world:=/path/to/world.sdf

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    description_dir = get_package_share_directory('antbot_description')
    bringup_dir = get_package_share_directory('antbot_bringup')

    # Default world (shipped inside antbot_bringup package)
    default_world = os.path.join(bringup_dir, 'worlds', 'depot.sdf')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the SDF world file')

    # Set IGN_GAZEBO_RESOURCE_PATH so Ignition can find package:// meshes
    # and model:// URIs from Gazebo model database and collections.
    resource_path = os.path.join(description_dir, os.pardir)
    gazebo_models = os.path.expanduser('~/.gazebo/models')
    collection_models = os.path.join(
        os.path.expanduser('~/ros2_ws'),
        'gazebo_models_worlds_collection', 'models')
    all_paths = [resource_path, gazebo_models, collection_models]
    existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing:
        all_paths.append(existing)
    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', ':'.join(all_paths))

    # Set IGN_GAZEBO_SYSTEM_PLUGIN_PATH so Ignition finds ign_ros2_control
    plugin_path = '/opt/ros/humble/lib'
    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    set_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        plugin_path + (':' + existing_plugin if existing_plugin else ''))

    # Process simulation URDF (uses IgnitionSystem instead of BoardInterface)
    urdf_path = os.path.join(description_dir, 'urdf', 'antbot_sim.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description_xml = robot_description_config.toxml()
    robot_description = {'robot_description': robot_description_xml}

    # Launch Ignition Gazebo
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen')

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'antbot',
            '-string', robot_description_xml,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}])

    # Controller spawners — delayed to wait for Gazebo sim loop to start.
    # The controller_manager inside Gazebo needs the sim loop running
    # before it can complete controller activation (switch).
    sim_yaml = os.path.join(bringup_dir, 'config',
                            'swerve_drive_controller_sim.yaml')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file', sim_yaml,
                   '--controller-manager-timeout', '30'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['antbot_swerve_controller',
                   '--param-file', sim_yaml,
                   '--controller-manager-timeout', '30'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    # Delay spawners: wait for Gazebo to fully start its sim loop
    delayed_spawners = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner])

    # Chain: JSB done -> swerve controller spawner
    swerve_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner]))

    # ros_gz_bridge: bridge Ignition topics to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan_0@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar_3d_points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            ('/imu', '/imu_node/imu/accel_gyro'),
            ('/lidar_3d_points/points', '/lidar_3d_points'),
        ],
        output='screen')

    return LaunchDescription([
        world_arg,
        set_resource_path,
        set_plugin_path,
        ign_gazebo,
        robot_state_publisher,
        spawn_robot,
        delayed_spawners,
        swerve_after_jsb,
        bridge,
    ])
