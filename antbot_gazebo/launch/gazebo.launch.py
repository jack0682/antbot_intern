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
# ANTBot Ignition Gazebo 시뮬레이션 런치 파일.
#
# 사용법:
#   ros2 launch antbot_gazebo gazebo.launch.py
#   ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
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


def _spawn_controllers(context, *args, **kwargs):
    """Gazebo 기동 후 컨트롤러를 순차적으로 스폰합니다."""
    import time
    delay = float(LaunchConfiguration('controller_delay').perform(context))
    time.sleep(delay)
    return []


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('antbot_gazebo')
    description_pkg = get_package_share_directory('antbot_description')

    # 기본 world 파일
    default_world = os.path.join(gazebo_pkg, 'worlds', 'empty.sdf')

    # ROS distro에 따른 플러그인 경로
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    plugin_path = os.path.join('/opt', 'ros', ros_distro, 'lib')

    # ── 런치 인자 ──
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world SDF 파일 경로')

    controller_delay_arg = DeclareLaunchArgument(
        'controller_delay',
        default_value='8.0',
        description='Gazebo 기동 대기 시간 (초)')

    # ── 환경변수 ──
    resource_path = os.path.join(description_pkg, os.pardir)
    existing_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        resource_path + (':' + existing_resource if existing_resource else ''))

    existing_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    set_plugin_path = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        plugin_path + (':' + existing_plugin if existing_plugin else ''))

    # ── URDF 처리 ──
    urdf_path = os.path.join(gazebo_pkg, 'urdf', 'antbot_sim.xacro')
    robot_description_xml = xacro.process_file(urdf_path).toxml()

    # ── Gazebo 실행 ──
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen')

    # ── 로봇 스폰 ──
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

    # ── 컨트롤러 설정 ──
    controller_yaml = os.path.join(
        gazebo_pkg, 'config', 'swerve_controller_gazebo.yaml')

    # Gazebo 기동 대기 후 컨트롤러 스폰
    wait_for_gazebo = OpaqueFunction(function=_spawn_controllers)

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

    # JSB 완료 후 스워브 컨트롤러 스폰
    swerve_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[swerve_spawner]))

    # ── Gazebo ↔ ROS 2 토픽 브릿지 ──
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
        # 인자
        world_arg,
        controller_delay_arg,
        # 환경변수
        set_resource_path,
        set_plugin_path,
        # Gazebo
        ign_gazebo,
        spawn_robot,
        robot_state_pub,
        # 컨트롤러 (대기 → JSB → 스워브 순차 스폰)
        wait_for_gazebo,
        jsb_spawner,
        swerve_after_jsb,
        # 토픽 브릿지
        gz_bridge,
    ])
