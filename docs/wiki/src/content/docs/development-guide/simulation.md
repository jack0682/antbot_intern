---
title: 5.4 시뮬레이션 환경 구축
description: Ignition Gazebo 기반 AntBot 시뮬레이션 환경 구축 가이드
sidebar:
  order: 4
---

AntBot은 Ignition Gazebo(Fortress)에서 실물 하드웨어 없이 4-wheel independent swerve-drive 로봇을 구동하고 테스트할 수 있습니다.
시뮬레이션 환경은 `antbot_gazebo` 패키지에서 제공합니다.

---

## 빠른 시작

### 의존성 설치

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher
```

### 빌드 및 실행

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to antbot_gazebo
source install/setup.bash
```

**Terminal 1 — Gazebo 시뮬레이션**

```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

:::note
Gazebo 창이 나타나고 컨트롤러가 로드될 때까지 약 8초가 소요됩니다.
GPU 성능이 낮은 환경에서는 `controller_delay`를 늘려야 합니다:
```bash
ros2 launch antbot_gazebo gazebo.launch.py controller_delay:=15.0
```
:::

**Terminal 2 — 텔레옵 (키보드 제어)**

```bash
ros2 run antbot_teleop teleop_keyboard
```

Nav2 자율주행 연동은 [5.3 Nav2 연동](/antbot/development-guide/navigation/)을 참고하세요.

### 런치 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `world` | `empty.sdf` | Gazebo world SDF 파일 경로 |
| `controller_delay` | `8.0` | Gazebo 기동 대기 시간 (초) |

---

## 시스템 아키텍처

### 시뮬레이션 파이프라인

```
사용자 입력 (텔레옵 / Nav2)
       │  /cmd_vel
       ▼
┌─ ROS 2 ───────────────────────────────────────┐
│  controller_manager (100Hz)                    │
│    ├── joint_state_broadcaster → /joint_states │
│    └── swerve_drive_controller                 │
│          ├── subscribes: /cmd_vel              │
│          ├── publishes:  /odom, /tf            │
│          └── command → IgnitionSystem          │
│  robot_state_publisher → /tf (base_link → *)  │
└────────────────────┬───────────────────────────┘
               ros_gz_bridge
┌────────────────────▼───────────────────────────┐
│  Ignition Gazebo (Fortress)                    │
│    ├── gpu_lidar (전방) → /scan_0              │
│    ├── gpu_lidar (후방) → /scan_1              │
│    ├── IMU → /imu/data                         │
│    └── IgnitionSystem (ros2_control HW plugin) │
└────────────────────────────────────────────────┘
```

### 런치 시퀀스

```
gazebo.launch.py
  ├── 환경변수 설정 (IGN_GAZEBO_RESOURCE_PATH, PLUGIN_PATH)
  ├── xacro → URDF 생성
  ├── ign gazebo -r world.sdf
  ├── 로봇 스폰 (x=0, y=0, z=0.5)
  ├── robot_state_publisher
  ├── [controller_delay 대기]
  ├── joint_state_broadcaster 스폰
  ├── swerve_drive_controller 스폰 (JSB 완료 후)
  └── ros_gz_bridge (센서 토픽 브릿지)
```

### 시뮬레이션 vs 실제 HW

| 항목 | 실제 HW | Gazebo |
|------|---------|--------|
| 하드웨어 인터페이스 | BoardInterface | **IgnitionSystem** |
| 가속 명령 | 지원 | **미지원** (velocity/position만) |
| 2D LiDAR | COIN D4 | **gpu_lidar** (ogre2 필수) |
| IMU | antbot_imu 노드 | **Gazebo IMU 플러그인** |
| 제어 주기 | 20 Hz | **100 Hz** |
| 스크럽 보정 | 필요 | **불필요** |
| IK 반복 횟수 | 0 | **3** (55mm 오프셋 보정) |
| Odom 적분 | rk4 | **analytic_swerve** |
| Odom 스무딩 | window: 1 | **window: 10** |

---

## URDF 구성

### 파일 구조

```
antbot_sim.xacro (시뮬레이션 진입점)
  ├── antbot_description/  (공유 — 실제 HW와 동일)
  │    ├── sensors.xacro, base.xacro, wheel.xacro
  └── antbot_gazebo/  (시뮬 전용)
       ├── ros2_control_gazebo.xacro   IgnitionSystem
       └── gazebo_plugins.xacro        마찰 + 센서
```

### 하드웨어 인터페이스

```
IgnitionSystem (ign_ros2_control)
  ├── 휠 (x4):     velocity cmd → velocity/position state
  └── 스티어링 (x4): position cmd → position/velocity state
```

:::caution
IgnitionSystem은 `acceleration`과 `effort` command interface를 지원하지 않습니다.
컨트롤러 설정에서 반드시 `use_acceleration_command: false`로 설정해야 합니다.
:::

### 마찰 설정

| 파트 | mu1 / mu2 | 설명 |
|------|-----------|------|
| base_link | 0.2 / 0.2 | 낮은 마찰 (슬라이딩) |
| wheel (x4) | 1.8 / 1.8 | 높은 마찰 (접지력) |
| steering (x4) | 0.0 / 0.0 | 마찰 없음 (자유 회전) |

### 센서

| 센서 | 토픽 | 샘플 수 | 범위 | 노이즈 |
|------|------|---------|------|--------|
| 전방 2D LiDAR | `/scan_0` | 720 | 0.6-20m | stddev 0.008 |
| 후방 2D LiDAR | `/scan_1` | 720 | 0.6-20m | stddev 0.008 |
| IMU | `/imu/data` | — | — | 각속도 0.0003, 가속도 0.02 |

:::caution
gpu_lidar는 반드시 **ogre2** 렌더 엔진에서만 동작합니다.
World SDF에서 `<render_engine>ogre2</render_engine>`으로 설정하세요.
:::

---

## 컨트롤러 설정

설정 파일: `antbot_gazebo/config/swerve_controller_gazebo.yaml`

| 파라미터 | 시뮬 값 | HW 값 | 차이 이유 |
|----------|---------|-------|-----------|
| `non_coaxial_ik_iterations` | **3** | 0 | Gazebo가 55mm 오프셋 오차를 노출 |
| `enable_steering_scrub_compensator` | **false** | true | Gazebo에 실제 스크럽 없음 |
| `velocity_rolling_window_size` | **10** | 1 | 시뮬 encoder 노이즈 평활화 |
| `odom_integration_method` | **analytic_swerve** | rk4 | piecewise-constant에 정확 |
| `use_acceleration_command` | **false** | true | IgnitionSystem 미지원 |

:::tip
`non_coaxial_ik_iterations`를 0으로 설정하면 스티어링-휠 간 55mm 오프셋으로 인한 오도메트리 드리프트가 발생합니다. 시뮬에서는 반드시 2~3으로 설정하세요.
:::

---

## 커스텀 World

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

World SDF 필수 요소:
- `render_engine: ogre2` (gpu_lidar 동작 필수)
- Physics, Sensors, UserCommands, SceneBroadcaster 플러그인
- ground_plane + sun(조명)

---

## 트러블슈팅

### gpu_lidar가 range_min만 반환

`render_engine`이 `ogre2`인지 확인. `ogre`로 설정하면 모든 레이가 최소값만 반환합니다.

### 컨트롤러 스폰 타임아웃

`controller_delay` 인자를 늘려보세요: `controller_delay:=15.0`

### URDF 모델만 확인

```bash
ros2 launch antbot_description description.launch.py
```

설치 및 빌드 방법은 [소프트웨어 환경 구축](/antbot/software/environment-setup/)을 참조하세요.
