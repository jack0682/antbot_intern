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
# Author: Jaehong Oh

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('antbot_navigation')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        choices=['sim', 'real'],
        description='Operating mode: sim (Gazebo) or real (physical robot)')

    mode = LaunchConfiguration('mode')

    # Resolve config paths based on mode: config/sim/ or config/real/
    config_dir = PathJoinSubstitution([pkg_dir, 'config', mode])
    nav2_params_file = PathJoinSubstitution([config_dir, 'nav2_params.yaml'])
    ekf_params_file = PathJoinSubstitution([config_dir, 'ekf.yaml'])
    slam_params_file = PathJoinSubstitution(
        [config_dir, 'slam_toolbox_params.yaml'])

    # use_sim_time is derived from mode: sim=true, real=false
    use_sim_time = PythonExpression(["'", mode, "' == 'sim'"])

    # Disable swerve controller's odom TF so the EKF publishes it instead.
    # Delayed 3s to ensure the swerve controller is fully active first.
    disable_odom_tf = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'param', 'set',
                 '/antbot_swerve_controller', 'enable_odom_tf', 'false'],
            output='screen')])

    # EKF node for sensor fusion (wheel odom + IMU)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}])

    # SLAM Toolbox (provides map -> odom TF instead of AMCL)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}])

    # Planner server (NavFn A*)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Smoother server (smooths global path before controller follows it)
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Controller server (MPPI holonomic)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Behavior server (spin, backup, wait)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Lifecycle manager for navigation nodes
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}])

    # Delay Nav2 nodes slightly to allow EKF and SLAM to initialize
    delayed_nav2 = TimerAction(
        period=3.0,
        actions=[
            planner_server_node,
            smoother_server_node,
            controller_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_navigation,
        ])

    return LaunchDescription([
        mode_arg,
        disable_odom_tf,
        ekf_node,
        slam_toolbox_node,
        delayed_nav2,
    ])
