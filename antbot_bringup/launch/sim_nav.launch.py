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
# Combined Ignition Gazebo simulation + Nav2 navigation launch for ANTBot.
# Starts Gazebo with a pre-built world/map and the full Nav2 stack for
# autonomous navigation without SLAM.
#
# Usage:
#   ros2 launch antbot_bringup sim_nav.launch.py
#   ros2 launch antbot_bringup sim_nav.launch.py world:=<world.sdf> map:=<map.yaml>
#   ros2 launch antbot_bringup sim_nav.launch.py use_ekf:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    description_dir = get_package_share_directory('antbot_description')
    bringup_dir = get_package_share_directory('antbot_bringup')
    nav_dir = get_package_share_directory('antbot_navigation')

    nav2_params_file = os.path.join(nav_dir, 'config', 'sim', 'nav2_params.yaml')
    ekf_params_file = os.path.join(nav_dir, 'config', 'sim', 'ekf.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='/home/ros2_ws/worlds/simple.sdf',
        description='Path to the SDF world file')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/ros2_ws/worlds/maps/simple.yaml',
        description='Full path to the Nav2 map YAML file')

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='Use EKF sensor fusion for odom->base_link TF')

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    use_ekf = LaunchConfiguration('use_ekf')

    # ── Environment ───────────────────────────────────────────────────
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

    plugin_path = '/opt/ros/humble/lib'
    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    set_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        plugin_path + (':' + existing_plugin if existing_plugin else ''))

    # ── URDF ──────────────────────────────────────────────────────────
    urdf_path = os.path.join(description_dir, 'urdf', 'antbot_sim.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description_xml = robot_description_config.toxml()
    robot_description = {'robot_description': robot_description_xml}

    # ── Ignition Gazebo ───────────────────────────────────────────────
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen')

    # ── Robot spawn ───────────────────────────────────────────────────
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

    # ── Robot state publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}])

    # ── ros_gz_bridge ─────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan_0@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        remappings=[
            ('/imu', '/imu_node/imu/accel_gyro'),
        ],
        output='screen')

    # ── Controller spawners (delayed for Gazebo startup) ──────────────
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

    delayed_spawners = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner])

    swerve_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner]))

    # ── EKF (optional) ────────────────────────────────────────────────
    # When use_ekf:=true, disable swerve controller odom TF and run EKF
    disable_odom_tf = ExecuteProcess(
        cmd=['ros2', 'param', 'set',
             '/antbot_swerve_controller', 'enable_odom_tf', 'false'],
        output='screen',
        condition=IfCondition(use_ekf))

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        condition=IfCondition(use_ekf))

    # Delay disable_odom_tf + EKF until swerve controller is up
    delayed_ekf = RegisterEventHandler(
        OnProcessExit(
            target_action=swerve_controller_spawner,
            on_exit=[disable_odom_tf, ekf_node]))

    # ── Nav2 stack ────────────────────────────────────────────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file,
                    {'yaml_filename': map_yaml,
                     'use_sim_time': True}])

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}])

    # Delay Nav2 until controllers are ready (swerve controller publishes
    # odom->base_link, which Nav2 needs). Add extra time for EKF if enabled.
    delayed_nav2 = TimerAction(
        period=15.0,
        actions=[
            map_server_node,
            amcl_node,
            planner_server_node,
            smoother_server_node,
            controller_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_localization,
            lifecycle_manager_navigation,
        ])

    # ── Initial pose for AMCL ────────────────────────────────────────
    # AMCL's set_initial_pose races with map reception during activation.
    # Publish /initialpose after Nav2 has had time to fully start and
    # receive the map, so AMCL can localize immediately.
    set_initial_pose = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once',
                     '/initialpose',
                     'geometry_msgs/msg/PoseWithCovarianceStamped',
                     '{header: {frame_id: "map"}, '
                     'pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
                     'orientation: {w: 1.0}}, '
                     'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, '
                     '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
                     '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                     '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                     '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                     '0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}'],
                output='screen'),
        ])

    # ── Launch description ────────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        world_arg,
        map_arg,
        use_ekf_arg,
        # Environment
        set_resource_path,
        set_plugin_path,
        # Simulation
        ign_gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        # Controllers
        delayed_spawners,
        swerve_after_jsb,
        # EKF (conditional)
        delayed_ekf,
        # Nav2
        delayed_nav2,
        # AMCL initial pose (after Nav2 is fully up)
        set_initial_pose,
    ])
