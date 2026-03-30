---
title: 5.3 Nav2 연동
description: AntBot Nav2 네비게이션 가이드
sidebar:
  order: 3
---

`antbot_navigation` 패키지는 AntBot 스워브 드라이브 로봇을 위한 [Nav2](https://docs.nav2.org/) 네비게이션 스택 통합을 제공합니다.

## 개요

Nav2는 로봇을 A 지점에서 B 지점으로 장애물을 피하며 자율 이동시키는 ROS 2 공식 네비게이션 프레임워크입니다. AntBot에서는 홀로노믹 스워브 드라이브 특성에 맞게 MPPI 컨트롤러, OmniMotionModel AMCL, EKF 센서 퓨전을 사용합니다.

## 의존성 설치

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller
```

## 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select antbot_navigation
source install/setup.bash
```

## 네비게이션 모드

### SLAM (맵 생성 + 주행)

맵 없이 시작하여 환경을 탐색하면서 맵을 생성합니다.

```bash
# Gazebo 시뮬레이션 위에서
ros2 launch antbot_navigation slam.launch.py mode:=sim

# 실제 로봇 위에서
ros2 launch antbot_navigation slam.launch.py mode:=real
```

맵 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Navigation (기존 맵 + 자율 주행)

저장된 맵과 AMCL 로컬라이제이션으로 완전한 자율 주행을 수행합니다.

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim \
  map:=/path/to/map.yaml
```

### Localization (위치 추정만)

자율 주행 전에 로컬라이제이션만 테스트할 때 사용합니다.

```bash
ros2 launch antbot_navigation localization.launch.py mode:=sim \
  map:=/path/to/map.yaml
```

## Sim / Real 모드

모든 launch 파일은 `mode` 인자로 설정 디렉토리와 `use_sim_time`을 자동 선택합니다:

| 설정 | `mode:=sim` | `mode:=real` |
|------|-------------|--------------|
| Config 디렉토리 | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 5.0 m/s | 2.0 m/s |
| Velocity smoother | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF 프로세스 노이즈 | 낮음 (이상적 센서) | 높음 (실제 노이즈) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

## 빠른 시작 (시뮬레이션)

시뮬레이션 환경 설정은 [5.4 시뮬레이션 환경 구축](/antbot/development-guide/simulation/)을 참고하세요.

**Terminal 1** — Gazebo:

```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

**Terminal 2** — SLAM 네비게이션:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim
```

**Terminal 3** — RViz:

```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

RViz에서:
1. **2D Pose Estimate**로 초기 위치 설정
2. **Nav2 Goal**로 목적지 클릭 → 자동 경로 계획 + 주행

## 아키텍처

```
         /cmd_vel (Twist)
              ▲
              │
┌─────────────┴──────────────────────────┐
│  Nav2 Stack                            │
│  BT Navigator → Planner → Controller  │
│  (A* path)      (MPPI Omni)           │
│        ↕            ↕                  │
│  Global Costmap  Local Costmap         │
│  (static+obs)    (obs+inflation)       │
│        ↕                               │
│  AMCL / SLAM Toolbox                   │
│  (map → odom TF)                       │
└────────────────────────────────────────┘
              │
              ▼
┌────────────────────────────────────────┐
│  EKF (robot_localization)              │
│  Input: /odom (vx,vy) + IMU (vyaw)    │
│  Output: odom → base_link TF (50Hz)   │
└────────────────────────────────────────┘
              │
              ▼
   Swerve Controller (IK → motor commands)
```

### TF 트리

```
map → odom → base_link → [sensor frames]
 ^       ^
 │       │
AMCL    EKF (wheel odom + IMU fusion)
```

네비게이션 launch가 실행되면 스워브 컨트롤러의 `odom→base_link` TF 발행을 비활성화하고, EKF가 센서 퓨전을 통해 더 정확한 TF를 발행합니다.

## 스워브 드라이브 특화 설정

### MPPI 컨트롤러

DWB 대신 MPPI를 사용하는 이유: 스워브의 조향 재정렬 지연과 불연속 모드 전환을 rollout 기반 최적화가 더 잘 처리합니다.

| 파라미터 | 값 | 목적 |
|----------|-----|------|
| `motion_model` | `Omni` | 홀로노믹 3-DOF (vx, vy, wz) |
| `vy_max` | 1.0 (sim) / 0.5 (real) | 횡이동 제한 (crab-walk 방지) |
| `PreferForwardCritic` | weight 5.0 | 전진 방향 선호 |
| `TwirlingCritic` | weight 5.0 | 불필요한 회전 억제 |

### EKF 센서 퓨전

| 소스 | 퓨전 상태 |
|------|----------|
| Wheel odom (`/odom`) | vx, vy, vyaw |
| IMU | yaw, vyaw (differential mode) |

### 듀얼 LiDAR 코스트맵

- 전방 LiDAR (`/scan_0`): AMCL/SLAM 입력 + 코스트맵
- 후방 LiDAR (`/scan_1`): 코스트맵만 (후진 안전)

### AMCL

`OmniMotionModel` 사용 — 스워브의 횡이동을 위치 추정에 반영합니다. 기본 `DifferentialMotionModel`을 사용하면 정확도가 크게 저하됩니다.

## 센서 토픽

| 토픽 | 타입 | 소스 |
|------|------|------|
| `/scan_0` | `LaserScan` | 전방 2D LiDAR |
| `/scan_1` | `LaserScan` | 후방 2D LiDAR |
| `/odom` | `Odometry` | 스워브 컨트롤러 |
| `/imu_node/imu/accel_gyro` | `Imu` | IMU 센서 |

시뮬레이션(ros_gz_bridge)과 실제 로봇(물리 드라이버) 모두 동일한 토픽명을 사용합니다.

## 문제 해결

- **"Failed to create plan"** — 초기 위치 오류. RViz에서 **2D Pose Estimate**로 수정
- **벽 충돌** — `inflation_radius` 또는 `collision_margin_distance` 증가
- **충돌 후 odom 드리프트** — EKF의 `odom0_rejection_threshold`가 스파이크 필터링. 지속 시 재로컬라이제이션
- **TF 진동** — 네비게이션 launch가 자동으로 스워브 컨트롤러 TF를 비활성화

현재 AntBot의 `/cmd_vel` 및 `/odom` 토픽 명세는 [주요 ROS 토픽/서비스](/antbot/development-guide/ros-topics/)에서 확인할 수 있습니다.
