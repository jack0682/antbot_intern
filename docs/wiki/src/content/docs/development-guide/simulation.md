---
title: 5.4 시뮬레이션 환경 구축
description: Ignition Gazebo 기반 AntBot 시뮬레이션 환경
sidebar:
  order: 4
---

`antbot_gazebo` 패키지는 실물 로봇 없이 AntBot 소프트웨어를 테스트할 수 있는 Ignition Gazebo 시뮬레이션 환경을 제공합니다.

## 의존성 설치

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher
```

## 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select antbot_gazebo
source install/setup.bash
```

## 실행

### Gazebo 시뮬레이션 기동

```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

Gazebo 창이 나타나고 컨트롤러가 스폰될 때까지 약 15초 대기합니다.

커스텀 월드 사용:

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

### 텔레오프로 수동 조종

```bash
ros2 run antbot_teleop teleop_keyboard
```

### Nav2 네비게이션과 함께 실행

시뮬레이션 위에서 자율 주행을 테스트하려면 [5.3 Nav2 연동](/antbot/development-guide/navigation/)을 참고하세요.

## 패키지 구조

```
antbot_gazebo/
├── config/
│   └── swerve_controller_gazebo.yaml   # Sim-specific controller params
├── launch/
│   └── gazebo.launch.py                # Gazebo + robot spawn + controllers
├── urdf/
│   ├── antbot_sim.xacro                # Top-level sim URDF
│   ├── gazebo_plugins.xacro            # Sensor plugins (LiDAR, IMU)
│   └── ros2_control_gazebo.xacro       # IgnitionSystem hardware interface
└── worlds/
    ├── empty.sdf                       # Empty world
    └── depot.sdf                       # Warehouse-style world
```

## 시뮬레이션 URDF

`antbot_gazebo/urdf/antbot_sim.xacro`는 `antbot_description`의 공유 매크로(base, wheel, sensors)를 그대로 사용하고, 하드웨어 인터페이스와 Gazebo 플러그인만 자체 정의합니다.

| 구분 | 실제 로봇 | 시뮬레이션 |
|------|----------|-----------|
| ros2_control 플러그인 | `BoardInterface` | `IgnitionSystem` |
| 명령 인터페이스 | velocity + acceleration | velocity only |
| 센서 | 물리 드라이버 | Gazebo gpu_lidar + IMU |

## 시뮬레이션 센서

| 센서 | 토픽 | 타입 | 주기 |
|------|------|------|------|
| 전방 2D LiDAR | `/scan_0` | `LaserScan` | 15 Hz |
| 후방 2D LiDAR | `/scan_1` | `LaserScan` | 15 Hz |
| IMU | `/imu/data` | `Imu` | 100 Hz |
| Clock | `/clock` | `Clock` | — |

`ros_gz_bridge`를 통해 Ignition 토픽이 ROS 2 토픽으로 브릿지됩니다.

## 컨트롤러 설정

`swerve_controller_gazebo.yaml`은 IgnitionSystem에 맞게 조정된 설정입니다:

- `use_acceleration_command: false` — IgnitionSystem이 acceleration 인터페이스를 지원하지 않음
- `update_rate: 100` Hz — 시뮬레이션 안정성을 위해 실제 HW(20Hz)보다 높음
- `enable_steering_scrub_compensator: false` — Gazebo에 실제 scrub 마찰 없음
- `odom_integration_method: "analytic_swerve"` — 시뮬레이션의 constant-velocity 구간에 정확

## Launch 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `world` | `worlds/empty.sdf` | Gazebo world SDF 파일 경로 |
| `controller_delay` | `8.0` | Gazebo 기동 대기 시간 (초) |

## 문제 해결

### gpu_lidar가 range_min만 반환

월드 SDF 파일의 `render_engine`이 `ogre2`인지 확인하세요. Ignition gpu_lidar는 ogre2에서만 정상 동작합니다.

### 컨트롤러 스폰 타임아웃

`controller_delay` 인자를 늘려보세요:

```bash
ros2 launch antbot_gazebo gazebo.launch.py controller_delay:=15.0
```

### URDF 모델만 확인

Gazebo 없이 URDF만 시각화하려면:

```bash
ros2 launch antbot_description description.launch.py
```

설치 및 빌드 방법은 [소프트웨어 환경 구축](/antbot/software/environment-setup/)을 참조하세요.
