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

:::note[ARM vs x86 빌드]
`antbot_camera`와 `antbot_bringup`은 ARM 전용 라이브러리에 의존하므로, x86 환경에서는 이 두 패키지를 제외하고 빌드합니다.
- **ARM (Jetson 등)**: `colcon build`로 전체 빌드 가능
- **x86 (일반 PC)**: `antbot_camera`, `antbot_bringup`을 제외하고 빌드

```bash
# x86 환경 — antbot_camera, antbot_bringup 제외
colcon build --symlink-install \
  --packages-ignore antbot_camera antbot_bringup
```
:::

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to antbot_gazebo
source install/setup.bash
```

---

## 실행 순서

**Terminal 1 — Gazebo 시뮬레이션**

```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

:::note
Gazebo 창이 나타난 후, 로봇 스폰이 완료되면 컨트롤러가 자동으로 활성화됩니다.
:::

**Terminal 2 — 텔레옵 (키보드 제어)**

```bash
ros2 run antbot_teleop teleop_keyboard
```

Nav2 자율주행 연동은 [5.3 Nav2 연동](/antbot/development-guide/navigation/)을 참고하세요.

### 런치 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `world` | `empty` | 월드 이름 (`worlds.yaml` 기준) 또는 SDF 파일 전체 경로 |

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
  ├── antbot_gazebo/config/worlds.yaml로 월드 이름 → SDF 경로 해석
  ├── ign gazebo -r world.sdf
  ├── 로봇 스폰 (x=0, y=0, z=0.5)
  ├── robot_state_publisher
  ├── [spawn_robot 종료 대기 (OnProcessExit)]
  ├── controller_manager 서비스 대기
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

| 파트 | 마찰 계수 | 설명 |
|------|-----------|------|
| base_link | 0.2 | 낮은 마찰 (슬라이딩) |
| wheel (x4) | 1.8 | 높은 마찰 (접지력) |
| steering (x4) | 0.0 | 마찰 없음 (자유 회전) |

### 센서

| 센서 | Gazebo 플러그인 | 토픽 | 샘플 수 | 범위 | 노이즈 |
|------|-----------------|------|---------|------|--------|
| 전방 2D LiDAR | `gpu_lidar` | `/scan_0` | 720 | 0.6-20m | stddev 0.008 m |
| 후방 2D LiDAR | `gpu_lidar` | `/scan_1` | 720 | 0.6-20m | stddev 0.008 m |
| IMU | `imu_sensor` | `/imu/data` | — | — | 각속도 stddev 0.0003 rad/s, 가속도 stddev 0.02 m/s² |

:::caution
`gpu_lidar`는 Gazebo에서 렌더링 엔진 기반으로 동작하며, Fortress에서는 보통 ogre2를 사용합니다. 기본 설정에서는 문제없이 동작하지만, 렌더 엔진을 ogre2가 아닌 값으로 바꾸면 GPU LiDAR 출력이 비정상적이거나 나오지 않을 수 있습니다.
:::

---

## 컨트롤러 설정

설정 파일: `antbot_gazebo/config/swerve_controller_gazebo.yaml`

| 파라미터 | 시뮬 값 | HW 값 | 차이 이유 |
|----------|---------|-------|-----------|
| `non_coaxial_ik_iterations` | **3** | 0 | 55mm 오프셋 오차 보정 필요 |
| `enable_steering_scrub_compensator` | **false** | true | 시뮬에서 스크럽 미발생 |
| `velocity_rolling_window_size` | **10** | 1 | 시뮬 encoder 노이즈 평활화 |
| `odom_integration_method` | **analytic_swerve** | rk4 | piecewise-constant에 정확 |
| `use_acceleration_command` | **false** | true | IgnitionSystem 미지원 |

---

## 커스텀 World

`antbot_gazebo/config/worlds.yaml`에 등록된 월드 이름 또는 SDF 파일 경로를 지정할 수 있습니다.

```bash
# 이름으로 실행 (worlds.yaml에서 SDF 경로를 자동 해석)
ros2 launch antbot_gazebo gazebo.launch.py world:=depot

# 전체 경로로 실행
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

### worlds.yaml 구성

시뮬레이션(SDF)과 네비게이션(맵)의 worlds.yaml이 분리되어 있습니다:

- **`antbot_gazebo/config/worlds.yaml`** — 월드 이름 → SDF 파일 매핑 (Gazebo용)
- **`antbot_navigation/maps/worlds.yaml`** — 월드 이름 → 맵 파일 매핑 (Nav2용)

```yaml
# antbot_gazebo/config/worlds.yaml
worlds:
  empty:
    sdf: empty.sdf
  depot:
    sdf: depot.sdf        # antbot_gazebo/worlds/ 기준
```

새 월드를 추가하려면 SDF 파일을 `antbot_gazebo/worlds/`에 배치하고, `worlds.yaml`에 항목을 추가하세요. Nav2에서도 사용하려면 `antbot_navigation/maps/worlds.yaml`에 맵 파일도 등록해야 합니다.

### World SDF 필수 요소

- `render_engine: ogre2` (gpu_lidar 동작 필수)
- Physics, Sensors, UserCommands, SceneBroadcaster 플러그인
- ground_plane + sun(조명)

---

## 트러블슈팅

:::caution[트러블슈팅]
**gpu_lidar가 range_min만 반환**
`render_engine`이 `ogre2`인지 확인하세요. `ogre`로 설정하면 모든 레이가 최소값만 반환합니다.

**컨트롤러 활성화 실패**
`ros2 control list_controllers`로 컨트롤러 상태를 확인하세요. `unconfigured` 상태에서 멈춰 있다면 `gz_ros2_control` 플러그인이 정상 로드되었는지 Gazebo 로그를 확인합니다.

**오도메트리 드리프트 발생**
`non_coaxial_ik_iterations`를 0으로 설정하면 스티어링-휠 간 55mm 오프셋으로 인한 드리프트가 발생합니다. 시뮬에서는 2~3으로 설정하세요.

**URDF 모델만 확인**
`ros2 launch antbot_description description.launch.py`
:::

설치 및 빌드 방법은 [소프트웨어 환경 구축](/antbot/software/environment-setup/)을 참조하세요.
