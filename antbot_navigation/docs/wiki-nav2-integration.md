---
title: "5.3 Nav2 연동"
description: "AntBot 스워브 드라이브 로봇의 Nav2 네비게이션 스택 연동 가이드"
---

# Nav2 연동

AntBot은 ROS 2의 공식 네비게이션 프레임워크인 Nav2를 통해 자율주행을 수행합니다.
4-wheel independent swerve-drive 특성에 맞춘 별도 튜닝이 적용되어 있으며, 이 문서에서는 시스템 구성, 파라미터 설정, 스워브 전용 튜닝 방법을 다룹니다.

---

## 개요

### Nav2 파이프라인

```
사용자가 목표 설정
       │
       ▼
┌─ BT Navigator ─────────────────────────────────┐
│  행동 트리로 전체 흐름 오케스트레이션             │
│  "계획 실패? → 스핀 → 재시도 → 대기 → 포기"     │
│                                                  │
│  ┌──────────────┐    ┌───────────────────┐      │
│  │ Planner      │    │ Controller        │      │
│  │ Server       │───▶│ Server            │      │
│  │              │    │                   │      │
│  │ "어디로      │    │ "어떻게           │      │
│  │  갈 것인가"  │    │  따라갈 것인가"   │      │
│  │              │    │                   │      │
│  │ NavFn(A*)    │    │ MPPI Omni         │      │
│  └──────┬───────┘    └────────┬──────────┘      │
│         │                     │                  │
│  ┌──────▼───────┐    ┌───────▼──────────┐       │
│  │ Global       │    │ Local            │       │
│  │ Costmap      │    │ Costmap          │       │
│  │ (정적맵+장애물│    │ (실시간 장애물    │       │
│  │  +inflation) │    │  +inflation)     │       │
│  └──────────────┘    └──────────────────┘       │
│                                                  │
│  ┌──────────────┐                               │
│  │ Behavior     │  spin, backup, wait           │
│  │ Server       │  "막혔을 때 복구 행동"         │
│  └──────────────┘                               │
└──────────────────────────────────────────────────┘
```

### AntBot 스워브 드라이브 vs 일반 diff-drive

| 항목 | 일반 diff-drive | AntBot swerve |
|------|-----------------|---------------|
| Controller | DWB | **MPPI** (rollout 기반 최적화) |
| Motion Model (AMCL) | DifferentialMotionModel | **OmniMotionModel** |
| 속도 자유도 | vx, wz (2DOF) | **vx, vy, wz (3DOF)** |
| Costmap Inflation | ~0.3m | **0.75m** (steering 오버슈트 대비) |
| LiDAR | 단일 | **듀얼 2D + 3D** |
| Odometry TF | 직접 발행 | **EKF 센서 퓨전** (충돌 방어) |

---

## 빠른 시작

### 의존성 설치

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller ros-humble-ros-gz \
  ros-humble-ign-ros2-control ros-humble-xacro
```

### 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 실행 (3개 터미널)

**Terminal 1 — Gazebo 시뮬레이션**

```bash
source install/setup.bash
ros2 launch antbot_bringup sim.launch.py
```

:::note
Gazebo 창이 나타나고 컨트롤러가 로드될 때까지 약 15초가 소요됩니다.
컨트롤러가 active 상태가 되어야 `/cmd_vel` 토픽 구독이 시작됩니다.
:::

**Terminal 2 — Nav2 네비게이션**

```bash
source install/setup.bash
ros2 launch antbot_navigation navigation.launch.py \
  map:=$(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/maps/depot_sim.yaml \
  use_sim_time:=true
```

**Terminal 3 — RViz 시각화**

```bash
source install/setup.bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

### 로봇 이동시키기

#### 단일 목표

1. RViz에서 **2D Pose Estimate**로 초기 위치 설정
2. **Nav2 Goal** 클릭 후 목적지 클릭

#### 웨이포인트 순회

1. RViz 메뉴 → **Panels** → **Add New Panel** → `nav2_rviz_plugins/Navigation2`
2. Nav2 패널에서 **Waypoint / Nav Through Poses Mode** 체크
3. **Nav2 Goal**로 여러 지점을 순서대로 클릭
4. Nav2 패널의 **Start Waypoint Following** 클릭

#### CLI로 이동

```bash
# 단일 목표
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"

# 웨이포인트 순회
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}},
    {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 4.0}}},
    {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 1.0}}}
  ]}"
```

### 실행 모드

| Launch 파일 | 용도 | 특징 |
|-------------|------|------|
| `navigation.launch.py` | 저장된 맵으로 자율주행 | AMCL + Nav2 전체 스택 |
| `slam.launch.py` | 맵 생성하면서 네비게이션 | SLAM Toolbox (맵 불필요) |
| `localization.launch.py` | 로컬라이제이션 전용 | 경로계획 없음, 위치 추정만 |

맵 저장: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`

---

## 시스템 아키텍처

### 노드 그래프

```
┌─────────────────────────── Gazebo Ignition ───────────────────────────────┐
│  IgnitionSystem (ros2_control)                                            │
│    ├── joint_state_broadcaster → /joint_states                            │
│    └── antbot_swerve_controller                                           │
│          ├── subscribes: /cmd_vel                                         │
│          └── publishes:  /odom, /tf (odom→base_link, 비활성화 가능)        │
│                                                                           │
│  gpu_lidar (front) → /scan_0          ┐                                   │
│  gpu_lidar (back)  → /scan_1          ├── ros_gz_bridge로 ROS 2 전달      │
│  gpu_lidar (3D)    → /lidar_3d_points │                                   │
│  IMU sensor        → /imu             ┘                                   │
└───────────────────────────────────────────────────────────────────────────┘
         │
    ros_gz_bridge (topic bridging + remapping)
         │
         ▼
┌─────────────────────────── Nav2 Navigation Stack ─────────────────────────┐
│                                                                           │
│  EKF (robot_localization)                                                 │
│    ├── subscribes: /odom (vx,vy,vyaw), /imu_node/imu/accel_gyro (yaw)    │
│    └── publishes:  /odometry/filtered, /tf (odom→base_link)               │
│                                                                           │
│  AMCL                                                                     │
│    ├── subscribes: /scan_0, /map                                          │
│    └── publishes:  /tf (map→odom), /amcl_pose                             │
│                                                                           │
│  Map Server → /map                                                        │
│                                                                           │
│  Planner Server (NavFn A*) → /plan                                        │
│    └── Smoother Server (SimpleSmoother) → smoothed path                   │
│                                                                           │
│  Controller Server (MPPI Omni) → /cmd_vel                                 │
│    ├── subscribes: /plan, local_costmap                                   │
│    └── publishes:  /cmd_vel → antbot_swerve_controller                    │
│                                                                           │
│  Behavior Server (spin, backup, wait)                                     │
│  BT Navigator (행동 트리 오케스트레이션)                                    │
│                                                                           │
│  Costmaps:                                                                │
│    ├── Global: static_layer + obstacle(scan_0,scan_1) + inflation(0.75m)  │
│    └── Local:  obstacle(scan_0,scan_1) + inflation(0.75m), 5x5m rolling   │
└───────────────────────────────────────────────────────────────────────────┘
```

### TF 트리

```
       map
        │
        │  ← AMCL (particle filter localization)
        ▼
       odom
        │
        │  ← EKF (wheel odom + IMU fusion)
        │     또는 swerve controller (standalone sim)
        ▼
     base_link
        │
        ├── lidar_2d_front_link   (x=0.320, z=0.252, yaw=180deg)
        ├── lidar_2d_back_link    (x=-0.320, z=0.252, yaw=0)
        ├── lidar_3d_link         (x=0.225, z=0.721, pitch=10deg)
        ├── imu_link              (x=0.240, z=0.379)
        ├── front_left_steering_link → front_left_wheel_link
        ├── front_right_steering_link → front_right_wheel_link
        ├── rear_left_steering_link → rear_left_wheel_link
        └── rear_right_steering_link → rear_right_wheel_link
```

#### TF 발행 주체

| 변환 | 발행자 | 비고 |
|------|--------|------|
| `map → odom` | AMCL 또는 SLAM Toolbox | localization |
| `odom → base_link` | EKF (Nav2 모드) 또는 swerve controller (standalone) | sensor fusion |
| `base_link → *` | robot_state_publisher | URDF joint states |

### odom TF 전환 메커니즘

```
sim.launch.py 단독 실행:
  swerve controller (enable_odom_tf: true) → odom→base_link TF 발행

navigation.launch.py 추가 실행:
  3초 후 ros2 param set enable_odom_tf false
  → swerve controller TF 발행 중단
  → EKF가 odom→base_link TF 발행 (센서 퓨전)
```

:::caution
swerve controller와 EKF가 동시에 `odom→base_link` TF를 발행하면 진동 현상이 발생합니다.
navigation launch가 3초 지연 후 자동으로 swerve controller의 TF 발행을 비활성화하지만,
타이밍 문제로 실패할 수 있습니다. 진동이 보이면 수동으로 비활성화하세요:

```bash
ros2 param set /antbot_swerve_controller enable_odom_tf false
```
:::

### 데이터 흐름: cmd_vel → 바퀴 회전

```
Nav2 MPPI Controller
  │
  │  /cmd_vel (geometry_msgs/Twist)
  │  vx, vy, wz in base_link frame
  ▼
Swerve Drive Controller
  │
  ├── process_cmd_and_limits()
  │   └── timeout guard, NaN guard, speed limiter
  │
  ├── compute_inverse_kinematics()
  │   ├── pivot velocity = cmd_vel + angular × module_position
  │   ├── non-coaxial IK refinement (2 iterations in sim)
  │   ├── steering angle = atan2(vy_contact, vx_contact)
  │   ├── wheel speed = ||v_contact|| / wheel_radius
  │   └── steering flip (180deg) if shorter path
  │
  ├── synchronized_motion_profile()
  │   └── 4 modules 동기화 (가장 느린 모듈에 맞춤)
  │
  └── command_steerings_and_wheels()
      ├── steering position command → Gazebo joint
      └── wheel velocity command → Gazebo joint
```

### 센서 토픽 요약

| 토픽 | 타입 | 주기 | 소스 |
|------|------|------|------|
| `/scan_0` | LaserScan | 10 Hz | 전방 2D LiDAR (360 samples, 0.1-20m) |
| `/scan_1` | LaserScan | 10 Hz | 후방 2D LiDAR (360 samples, 0.1-20m) |
| `/lidar_3d_points` | PointCloud2 | 10 Hz | 3D LiDAR (720x16, 0.05-70m) |
| `/odom` | Odometry | 50 Hz | swerve controller (wheel odometry) |
| `/imu_node/imu/accel_gyro` | Imu | 200 Hz | IMU 센서 |
| `/odometry/filtered` | Odometry | 50 Hz | EKF 출력 (퓨전된 odom) |
| `/cmd_vel` | Twist | 20 Hz | MPPI controller 출력 |

:::tip
`/cmd_vel`과 `/odom`의 상세 메시지 정의는 [5.2 주요 ROS 토픽/서비스](/antbot/development-guide/ros-topics/)를 참고하세요.
:::

---

## 파라미터 설정

설정 파일 위치: `antbot_navigation/config/`

### AMCL 로컬라이제이션

```yaml
amcl:
  robot_model_type: "nav2_amcl::OmniMotionModel"  # 홀로노믹 필수
  max_particles: 2000
  min_particles: 500
  scan_topic: /scan_0              # 전방 2D LiDAR
  set_initial_pose: true           # 자동 초기 위치 설정
  laser_model_type: "likelihood_field"
  max_beams: 60
  update_min_d: 0.25               # 스캔 업데이트 최소 이동 거리 (m)
  update_min_a: 0.2                # 스캔 업데이트 최소 회전 (rad)
```

:::caution
`OmniMotionModel`은 스워브 로봇에 **필수**입니다. 기본 `DifferentialMotionModel`을 사용하면 횡이동(vy)을 모델링하지 못해 localization 정확도가 크게 떨어집니다.
:::

### EKF 센서 퓨전

`ekf.yaml` — robot_localization 패키지의 Extended Kalman Filter로 바퀴 오도메트리와 IMU를 퓨전합니다.

```yaml
ekf_filter_node:
  frequency: 50.0          # 출력 주기 (Hz)
  two_d_mode: true         # 2D 평면 운동만 추정
  publish_tf: true         # odom→base_link TF 발행

  # Wheel odometry: 선속도 + 각속도 퓨전
  odom0: /odom
  odom0_config: [F,F,F, F,F,F, T,T,F, F,F,T, F,F,F]
  #                              vx vy     vyaw
  odom0_rejection_threshold: 2.0   # 충돌 스파이크 거부

  # IMU: heading + 각속도 퓨전
  imu0: /imu_node/imu/accel_gyro
  imu0_config: [F,F,F, F,F,T, F,F,F, F,F,T, F,F,F]
  #                       yaw           vyaw
  imu0_differential: true              # 절대값이 아닌 변화량 사용
```

스워브 드라이브의 **vy(횡속도)**를 EKF에 퓨전하는 것이 diff-drive와의 핵심 차이입니다. diff-drive는 vy=0을 가정하지만 스워브는 실제 vy가 존재합니다.

:::note[충돌 보호]
`odom0_rejection_threshold: 2.0`은 Mahalanobis distance 기반 outlier 거부입니다.
벽 충돌 시 바퀴 슬립으로 wheel velocity 스파이크가 발생하면 이 threshold로 자동 무시되어
odom 드리프트를 방지합니다.
:::

#### Process Noise Covariance

15x15 대각 행렬. 2D 모드에서 유효한 요소:

| 상태 | 값 | 설명 |
|------|-----|------|
| x, y | 0.05 | 위치 예측 신뢰도 |
| yaw | 0.02 | heading 예측 신뢰도 |
| vx, vy | 0.025 | 속도 예측 신뢰도 |
| vyaw | 0.01 | 각속도 예측 신뢰도 |

값이 클수록 측정값을 더 신뢰하고, 작을수록 예측 모델을 더 신뢰합니다.

### MPPI 컨트롤러

`nav2_params.yaml` — MPPI(Model Predictive Path Integral) 컨트롤러 설정.

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Omni"           # 홀로노믹 스워브
  time_steps: 56                 # rollout 스텝 수
  model_dt: 0.05                 # 스텝당 시간 (s) → 2.8초 미래 예측
  batch_size: 2000               # 후보 궤적 수
  temperature: 0.2               # 낮을수록 최적 궤적에 집중
  vx_max: 1.5                    # 직선 최대 속도 (m/s)
  vx_min: -0.3                   # 후진 최대 속도 (m/s)
  vy_max: 0.15                   # 횡방향 제한 (crab-walk 방지)
  wz_max: 1.5                    # 최대 회전 속도 (rad/s)

progress_checker:
  required_movement_radius: 0.3  # 이 거리 이상 이동해야 progress 인정
  movement_time_allowance: 20.0  # 이 시간 내 이동 못하면 abort

general_goal_checker:
  xy_goal_tolerance: 0.25        # 목표 도달 판정 (m)
  yaw_goal_tolerance: 0.25       # 목표 방향 판정 (rad)
```

:::note[progress_checker 튜닝 이력]
스워브의 steering re-alignment에 시간이 소요되어 기본값(radius: 0.5, time: 10.0)으로는 웨이포인트 순회 시
`Failed to make progress` 오류가 반복되었습니다. radius를 0.3으로 줄이고 time을 20.0으로 늘려 해결했습니다.
:::

#### MPPI Critics

| Critic | 가중치 | 역할 |
|--------|--------|------|
| ConstraintCritic | 4.0 | 속도/가속 제약 위반 페널티 |
| CostCritic | 3.81 | costmap 비용 반영 |
| GoalCritic | 5.0 | 목표 접근 보상 |
| GoalAngleCritic | 3.0 | 목표 도달 시 방향 정렬 |
| **PreferForwardCritic** | **15.0** | **전진 방향 선호 (스워브 핵심)** |
| ObstaclesCritic | - | 장애물 충돌 회피 |
| **PathAlignCritic** | **14.0** | 궤적이 글로벌 경로와 정렬 |
| PathFollowCritic | 5.0 | 글로벌 경로 추종 |
| **PathAngleCritic** | **20.0** | **body 각도 = 경로 방향 (스워브 핵심)** |
| **TwirlingCritic** | **15.0** | **불필요한 회전 억제 (스워브 핵심)** |

:::tip
볼드 표시된 4개 critic이 스워브 드라이브 전용 핵심 파라미터입니다. 이 값들이 낮으면 로봇이 crab-walking하거나 불필요하게 회전합니다.
:::

### Costmap 설정

Local과 Global costmap 모두 동일한 footprint과 obstacle/inflation 설정을 사용합니다.

```yaml
# 로봇 풋프린트 0.70m x 0.60m (local/global 양쪽에 동일 정의, keep in sync)
footprint: "[[0.35, 0.30], [0.35, -0.30], [-0.35, -0.30], [-0.35, 0.30]]"

obstacle_layer:
  data_type: "LaserScan"
  obstacle_min_range: 0.25       # 라이다 self-hit 데이터 필터링
  raytrace_min_range: 0.25
  obstacle_max_range: 4.5
  raytrace_max_range: 5.0

inflation_layer:
  inflation_radius: 0.75          # 장애물 주변 안전 거리 (m)
  cost_scaling_factor: 1.5        # 비용 감쇠율 (낮을수록 멀리까지 유지)
```

| 항목 | Local Costmap | Global Costmap |
|------|---------------|----------------|
| Frame | `odom` | `map` |
| 크기 | 5m x 5m rolling window | 전체 맵 |
| 해상도 | 0.05m | 0.05m |
| 업데이트 | 5Hz | 1Hz |
| 레이어 | obstacle(front,back) + inflation | static + obstacle(front,back) + inflation |

듀얼 2D LiDAR(전방 `/scan_0` + 후방 `/scan_1`)가 양쪽 costmap에 각각 별도의 obstacle layer로 등록되어 360도 장애물 감지를 제공합니다.

### Planner / Smoother / Behavior Server

```yaml
# Planner — NavFn A* (홀로노믹에 충분)
planner_server:
  GridBased:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    use_astar: true
    tolerance: 0.5          # 목표 근처 탐색 허용 범위 (m)
    allow_unknown: true     # 미탐색 영역 통과 허용

# Smoother — 격자 경로를 부드러운 곡선으로
smoother_server:
  simple_smoother:
    plugin: "nav2_smoother::SimpleSmoother"
    max_its: 1000
    do_refinement: true

# Behavior Server — 복구 행동
behavior_server:
  behavior_plugins: ["spin", "backup", "wait"]
  max_rotational_vel: 1.0
  simulate_ahead_time: 2.0
```

### SLAM Toolbox (맵핑 모드)

```yaml
slam_toolbox:
  solver_plugin: solver_plugins::CeresSolver
  scan_topic: /scan_0
  mode: mapping
  resolution: 0.05                # 맵 해상도 (meters/pixel)
  max_laser_range: 20.0
  minimum_travel_distance: 0.5    # 스캔 삽입 최소 이동 거리 (m)
  minimum_travel_heading: 0.5     # 스캔 삽입 최소 회전 (rad)
  do_loop_closing: true           # 루프 클로저 활성화
```

---

## 스워브 드라이브 튜닝 가이드

일반 diff-drive와 다른 AntBot 스워브 드라이브만의 Nav2 튜닝 포인트입니다.

### 왜 MPPI인가? (DWB 대신)

| 항목 | DWB | MPPI |
|------|-----|------|
| 방식 | 단일 샘플 평가 | 2000개 궤적 rollout + 비용 최적화 |
| 모션 모델 | holonomic 가능하나 제한적 | `Omni` 모델 네이티브 지원 |
| steering 지연 대응 | 즉각 반응 가정 | rollout으로 지연 고려 가능 |
| discontinuous 전환 | 대응 어려움 | 여러 후보 중 안정적 궤적 선택 |

스워브 드라이브는 이론상 홀로노믹이지만 실제로는:
- 모드 전환 시 steering re-alignment이 필요
- jerk/acceleration/velocity limiting이 걸림
- 특정 전환에서 불연속 정지 후 재시작

이런 특성에 MPPI의 rollout 기반 최적화가 더 적합합니다. 2.8초 미래 예측(56 steps × 0.05s)으로 steering 지연을 고려한 궤적을 선택합니다.

### Body Heading 정렬 (가장 중요)

스워브는 바퀴가 독립 회전하므로 body heading과 이동 방향이 무관할 수 있습니다. Crab-walking(옆으로 걸음)을 방지하려면:

```yaml
# vy를 강하게 제한 (crab-walk 방지)
vy_max: 0.15          # 횡방향은 미세 조정용만 허용

# 전진 방향 선호 critic
PreferForwardCritic:
  cost_weight: 15.0    # 높을수록 전진 선호 강함

# 경로 방향과 body 각도 정렬
PathAngleCritic:
  cost_weight: 20.0    # 높을수록 body 회전하여 경로 방향 정렬

# 불필요한 회전 억제
TwirlingCritic:
  cost_weight: 15.0
```

:::note[튜닝 이력]
초기값에서 crab-walking이 심하게 발생했습니다. 아래와 같이 조정하여 해결:
- `vy_max`: 0.5 → **0.15**
- `PreferForwardCritic`: 5.0 → **15.0**
- `PathAngleCritic`: 2.0 → **20.0**
- `TwirlingCritic`: 10.0 → **15.0**
:::

### 적응형 속도 (직선 가속 / 커브 감속)

```yaml
vx_max: 1.5           # 직선에서 최대 속도 허용
temperature: 0.2       # 낮을수록 최적 궤적에 집중
```

PathAngleCritic(20.0)이 커브에서 고속 궤적에 자동 페널티를 부과하므로, 커브 진입 시 MPPI가 자연스럽게 낮은 속도 궤적을 선택합니다.

### Costmap Inflation 튜닝

```yaml
inflation_radius: 0.75         # 로봇 inscribed radius(0.30m) + 0.45m 여유
cost_scaling_factor: 1.5       # 낮을수록 장애물에서 멀리까지 비용 유지
```

일반 diff-drive보다 inflation이 넉넉해야 하는 이유:
- 스워브는 steering 전환 중 약간의 오버슈트 가능
- re-alignment 동안 정확한 궤적 추종이 안 됨
- 충분한 여유가 없으면 벽 근처에서 충돌

:::note[튜닝 이력]
초기 설정에서 벽 충돌이 반복되어 아래와 같이 조정:
- `inflation_radius`: 0.35 → **0.75** (양쪽 costmap)
- `cost_scaling_factor`: 3.0 → **1.5** (비용을 멀리까지 유지)
:::

### LiDAR Self-Hit 필터링

```yaml
obstacle_min_range: 0.25       # 라이다 self-hit 데이터 필터링
raytrace_min_range: 0.25       # 라이다가 본체에 맞는 ~0.22m 이하 제거
```

2D LiDAR가 로봇 본체 내부(x=±0.32m)에 위치하여 일부 레이가 body mesh에 맞습니다.
`obstacle_min_range`로 이 데이터를 필터링하지 않으면, 로봇 위치가 costmap에서 장애물 내부로 인식되어 경로 계획이 실패합니다.

### EKF 충돌 보호

```yaml
odom0_rejection_threshold: 2.0   # Mahalanobis distance 기반 outlier 거부
```

벽 충돌 시 바퀴 슬립으로 wheel velocity 스파이크가 발생합니다. 이 threshold로 비정상 측정값을 자동 거부하여 odom 드리프트를 방지합니다.

EKF 퓨전 구성:

```
Wheel Odom (/odom):   vx, vy, vyaw  ← 선속도 + 각속도
IMU:                  yaw, vyaw     ← 절대 heading + 각속도
```

스워브의 vy(횡속도)를 EKF에 퓨전하는 것이 diff-drive와의 차이입니다.

### 시뮬레이션 vs 하드웨어 파라미터

| 파라미터 | 실제 HW | Sim | 이유 |
|----------|---------|-----|------|
| `non_coaxial_ik_iterations` | 0 | 2 | Gazebo가 55mm 오프셋 오차를 노출 |
| `enable_steering_scrub_compensator` | true | false | Gazebo에 실제 scrub 없음 |
| `velocity_rolling_window_size` | 1 | 5 | sim encoder 노이즈 평활화 |
| `odom_integration_method` | rk4 | analytic_swerve | piecewise-constant에 정확 |
| `enable_odom_tf` | true | true* | *Nav2 실행 시 비활성화 |

:::tip
시뮬레이션 환경의 상세 설정은 [5.4 시뮬레이션 환경 구축](/antbot/development-guide/simulation/)을 참고하세요.
:::

### 튜닝 체크리스트

- [ ] RViz에서 local costmap에 장애물이 보이는가?
- [ ] 로봇이 벽 근처에서 충분한 거리를 유지하는가?
- [ ] 직선 구간에서 가속하고 커브에서 감속하는가?
- [ ] body heading이 이동 방향과 정렬되는가?
- [ ] 벽 충돌 후 odom이 안정적인가?
- [ ] 웨이포인트 순회 시 progress checker가 fail하지 않는가?

---

## 트러블슈팅

### 빌드/실행 문제

#### "package not found" 에러

```
PackageNotFoundError: "package 'antbot_camera' not found"
```

`bringup.launch.py`는 카메라 의존성이 있습니다. 시뮬레이션은 `sim.launch.py`를 사용하세요.

#### "Could not load mesh resource" (RViz)

```
Could not load mesh resource 'package://antbot_description/meshes/...'
```

`source install/setup.bash`를 먼저 실행해야 `package://` 경로를 resolve할 수 있습니다.

#### Gazebo gpu_lidar가 전부 range_min 반환

월드 SDF 파일의 Sensors 플러그인 `render_engine`이 `ogre2`인지 확인:

```xml
<!-- WRONG -->
<render_engine>ogre</render_engine>

<!-- CORRECT -->
<render_engine>ogre2</render_engine>
```

Ignition gpu_lidar는 `ogre2` 렌더 엔진에서만 정상 동작합니다.

---

### 네비게이션 문제

#### "Failed to create plan"

**원인**: 로봇이 costmap에서 장애물 내부에 위치.

**해결**:
1. RViz에서 **2D Pose Estimate**로 로봇 위치 수정
2. 그래도 안 되면 costmap 초기화:
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap
```

#### "Failed to make progress"

**원인**: 로봇이 `movement_time_allowance` 시간 내에 `required_movement_radius`만큼 이동하지 못함.

**해결**:
```yaml
progress_checker:
  required_movement_radius: 0.3   # 줄이기 (기본 0.5)
  movement_time_allowance: 20.0   # 늘리기 (기본 10.0)
```

#### 로봇이 벽에 충돌

**해결 (순서대로 시도)**:
1. `inflation_radius` 증가 (0.75 → 1.0)
2. `cost_scaling_factor` 감소 (1.5 → 1.0)
3. MPPI `ObstaclesCritic.collision_margin_distance` 증가
4. `vx_max` 감소 (커브에서 과속 방지)

#### 로봇이 crab-walking (옆으로 걸음)

**원인**: MPPI가 vy를 과도하게 사용.

**해결**: `vy_max`를 더 줄이거나 (0.15 → 0.05), `PreferForwardCritic` 가중치 증가.

#### 경로가 지그재그

**원인**: Path Smoother 미적용 또는 NavFn 해상도 문제.

**해결**: `smoother_server`가 `lifecycle_manager_navigation`에 등록되어 있는지 확인.

---

### TF/Odometry 문제

#### TF가 튀는 현상 (진동)

**원인**: swerve controller와 EKF가 동시에 `odom→base_link` TF를 발행.

**확인**:
```bash
ros2 param get /antbot_swerve_controller enable_odom_tf
```

`true`이면 수동으로 비활성화:
```bash
ros2 param set /antbot_swerve_controller enable_odom_tf false
```

#### "Sensor origin out of map bounds"

위의 TF 이중 발행이 원인입니다. 같은 해결법을 적용하세요.

#### 충돌 후 odom 드리프트

EKF의 `odom0_rejection_threshold: 2.0`이 대부분의 스파이크를 필터링합니다. 그래도 드리프트가 심하면:
1. RViz에서 **2D Pose Estimate**로 재로컬라이제이션
2. `velocity_rolling_window_size` 증가 (5 → 10)

---

### 센서 문제

#### /scan_0 데이터 없음

```bash
# Ignition 토픽 확인
ign topic -l | grep scan

# ROS 브릿지 확인
ros2 topic hz /scan_0
```

#### 라이다 데이터가 원형으로만 보임 (장애물 없음)

1. `render_engine`이 `ogre2`인지 확인 (가장 흔한 원인)
2. 라이다 `<min>`이 너무 높지 않은지 확인
3. 월드에 실제 벽/장애물 메시가 있는지 확인

---

### 진단 명령어 모음

```bash
# 토픽 상태 확인
ros2 topic list | grep -E "scan|odom|cmd_vel|cost"
ros2 topic hz /scan_0
ros2 topic hz /odom

# TF 확인
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 컨트롤러 상태
ros2 control list_controllers
ros2 control list_hardware_interfaces

# 파라미터 확인
ros2 param get /antbot_swerve_controller enable_odom_tf
ros2 param get /ekf_filter_node use_sim_time

# Costmap 확인
ros2 topic echo /local_costmap/costmap --once | head -20

# Nav2 lifecycle 상태
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
```
