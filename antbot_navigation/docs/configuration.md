# Configuration Reference

모든 설정 파일의 파라미터 상세 설명.

> **sim/real 모드 분리**: 설정 파일은 `config/sim/`과 `config/real/` 디렉토리로 분리되어 있다.
> Launch 파일의 `mode` 인자 (`mode:=sim` 또는 `mode:=real`)에 따라 해당 디렉토리의 설정이 자동 로드된다.
> 아래 설명의 모든 파라미터는 `config/{mode}/` 하위 파일에 정의되어 있다.

## nav2_params.yaml

경로: `config/{mode}/nav2_params.yaml` (mode = sim 또는 real)

### BT Navigator

```yaml
bt_navigator:
  odom_topic: /odometry/filtered    # EKF 출력 (센서 퓨전된 odom)
  navigators:
    - navigate_to_pose              # 단일 목표
    - navigate_through_poses        # 웨이포인트 순회
```

### Planner Server

```yaml
planner_server:
  GridBased:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    use_astar: true       # A* 알고리즘 (Dijkstra보다 빠름)
    tolerance: 0.5        # 목표 근처 탐색 허용 범위 (m)
    allow_unknown: true   # 미탐색 영역 통과 허용
```

### Smoother Server

```yaml
smoother_server:
  simple_smoother:
    plugin: "nav2_smoother::SimpleSmoother"
    max_its: 1000         # 반복 횟수 (높을수록 부드러움)
    do_refinement: true   # 2차 정제 패스
```

NavFn이 생성한 격자 기반 경로의 꺾임을 부드러운 곡선으로 변환.

### Controller Server (MPPI)

```yaml
controller_server:
  controller_frequency: 20.0     # cmd_vel 발행 주기 (Hz)

  progress_checker:
    required_movement_radius: 0.3  # 이 거리 이상 이동해야 progress 인정
    movement_time_allowance: 20.0  # 이 시간 내 이동 못하면 abort

  general_goal_checker:
    xy_goal_tolerance: 0.25        # 목표 도달 판정 (m)
    yaw_goal_tolerance: 0.25       # 목표 방향 판정 (rad)

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
```

#### MPPI Critics

| Critic | 가중치 | 역할 |
|--------|--------|------|
| ConstraintCritic | 4.0 | 속도/가속 제약 위반 페널티 |
| CostCritic | 3.81 | costmap 비용 반영 |
| GoalCritic | 5.0 | 목표 접근 보상 |
| GoalAngleCritic | 3.0 | 목표 도달 시 방향 정렬 |
| PreferForwardCritic | 15.0 | **전진 방향 선호 (스워브 핵심)** |
| ObstaclesCritic | - | 장애물 충돌 회피 |
| PathAlignCritic | 14.0 | 궤적이 글로벌 경로와 정렬 |
| PathFollowCritic | 5.0 | 글로벌 경로 추종 |
| PathAngleCritic | 20.0 | **body 각도 = 경로 방향 (스워브 핵심)** |
| TwirlingCritic | 15.0 | **불필요한 회전 억제 (스워브 핵심)** |

### Costmap (Local / Global 공통)

```yaml
# 로봇 풋프린트 0.70m x 0.60m (local/global 양쪽에 동일 정의, keep in sync)
footprint: "[[0.35, 0.30], [0.35, -0.30], [-0.35, -0.30], [-0.35, 0.30]]"

obstacle_layer:
  data_type: "LaserScan"
  obstacle_min_range: 0.25       # 라이다 self-hit 필터링
  raytrace_min_range: 0.25
  obstacle_max_range: 4.5
  raytrace_max_range: 5.0

inflation_layer:
  inflation_radius: 0.75          # 장애물 주변 안전 거리 (m)
  cost_scaling_factor: 1.5        # 비용 감쇠율 (낮을수록 멀리까지 유지)
```

Local costmap: 5m x 5m rolling window, 5Hz 업데이트
Global costmap: 전체 맵, 1Hz 업데이트

### Behavior Server

```yaml
behavior_server:
  behavior_plugins: ["spin", "backup", "wait"]
  max_rotational_vel: 1.0     # 복구 행동 시 최대 회전 속도 (rad/s)
  simulate_ahead_time: 2.0    # 행동 시뮬레이션 시간 (s)
```

### AMCL

```yaml
amcl:
  robot_model_type: "nav2_amcl::OmniMotionModel"  # 홀로노믹 필수
  max_particles: 2000
  min_particles: 500
  scan_topic: /scan_0              # 전방 2D LiDAR
  set_initial_pose: true           # 자동 초기 위치 설정
```

`OmniMotionModel`은 스워브 로봇에 필수. 기본 `DifferentialMotionModel`을 사용하면 횡이동을 모델링하지 못해 localization 정확도가 크게 떨어진다.

---

## ekf.yaml

경로: `config/{mode}/ekf.yaml` (mode = sim 또는 real)

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

### Process Noise Covariance

15x15 대각 행렬. State vector: `[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`

2D 모드에서 유효한 요소: x(0.05), y(0.05), yaw(0.02), vx(0.025), vy(0.025), vyaw(0.01)

값이 클수록 측정값을 더 신뢰 (예측을 덜 신뢰).

---

## slam_toolbox_params.yaml

경로: `config/{mode}/slam_toolbox_params.yaml` (mode = sim 또는 real)

```yaml
slam_toolbox:
  solver_plugin: solver_plugins::CeresSolver
  scan_topic: /scan_0
  mode: mapping

  resolution: 0.05                # 맵 해상도 (meters/pixel)
  max_laser_range: 20.0           # 라이다 최대 범위 (m)
  minimum_travel_distance: 0.5    # 스캔 삽입 최소 이동 거리 (m)
  minimum_travel_heading: 0.5     # 스캔 삽입 최소 회전 (rad)
  do_loop_closing: true           # 루프 클로저 활성화
```
