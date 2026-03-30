# Architecture

## 시스템 개요

AntBot Nav2 스택은 4-wheel independent swerve-drive 로봇을 위해 튜닝된 네비게이션 시스템이다. 일반적인 differential-drive Nav2 설정과 다른 핵심 차이점:

1. **MPPI 컨트롤러** (DWB 대신) — rollout 기반 최적화가 steering re-alignment 지연에 더 적합
2. **Omni 모션 모델** — (vx, vy, wz) 3자유도 홀로노믹
3. **EKF 센서 퓨전** — 바퀴 오도메트리 + IMU를 퓨전하여 충돌 시 odom 드리프트 방지
4. **듀얼 2D LiDAR + 3D LiDAR** — 전방/후방 2D + 상부 3D 포인트클라우드

## 노드 그래프

```
┌─────────────────────────────── Gazebo Ignition ───────────────────────────────┐
│  IgnitionSystem (ros2_control)                                                │
│    ├── joint_state_broadcaster → /joint_states                                │
│    └── antbot_swerve_controller                                               │
│          ├── subscribes: /cmd_vel                                              │
│          └── publishes:  /odom, /tf (odom→base_link, 비활성화 가능)             │
│                                                                               │
│  gpu_lidar (front) → /scan_0          ┐                                       │
│  gpu_lidar (back)  → /scan_1          ├── ros_gz_bridge로 ROS 2 전달          │
│  gpu_lidar (3D)    → /lidar_3d_points │                                       │
│  IMU sensor        → /imu             ┘                                       │
└───────────────────────────────────────────────────────────────────────────────┘
         │
    ros_gz_bridge (topic bridging + remapping)
         │
         ▼
┌─────────────────────────── Nav2 Navigation Stack ─────────────────────────────┐
│                                                                               │
│  EKF (robot_localization)                                                     │
│    ├── subscribes: /odom (vx,vy,vyaw), /imu_node/imu/accel_gyro (yaw,vyaw)   │
│    └── publishes:  /odometry/filtered, /tf (odom→base_link)                   │
│                                                                               │
│  AMCL                                                                         │
│    ├── subscribes: /scan_0, /map                                              │
│    └── publishes:  /tf (map→odom), /amcl_pose                                 │
│                                                                               │
│  Map Server → /map                                                            │
│                                                                               │
│  Planner Server (NavFn A*) → /plan                                            │
│    └── Smoother Server (SimpleSmoother) → smoothed path                       │
│                                                                               │
│  Controller Server (MPPI Omni) → /cmd_vel                                     │
│    ├── subscribes: /plan, local_costmap                                       │
│    └── publishes:  /cmd_vel → antbot_swerve_controller                        │
│                                                                               │
│  Behavior Server (spin, backup, wait)                                         │
│  BT Navigator (행동 트리 오케스트레이션)                                        │
│                                                                               │
│  Costmaps:                                                                    │
│    ├── Global: static_layer + obstacle(scan_0,scan_1) + inflation(0.75m)      │
│    └── Local:  obstacle(scan_0,scan_1) + inflation(0.75m), 5x5m rolling       │
└───────────────────────────────────────────────────────────────────────────────┘
```

## TF 트리

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

### TF 발행 주체

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

## 데이터 흐름: cmd_vel → 바퀴 회전

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

## 센서 토픽 요약

| 토픽 | 타입 | 주기 | 소스 |
|------|------|------|------|
| `/scan_0` | LaserScan | 10 Hz | 전방 2D LiDAR (360 samples, 0.1-20m) |
| `/scan_1` | LaserScan | 10 Hz | 후방 2D LiDAR (360 samples, 0.1-20m) |
| `/lidar_3d_points` | PointCloud2 | 10 Hz | 3D LiDAR (720x16, 0.05-70m) |
| `/odom` | Odometry | 50 Hz | swerve controller (wheel odometry) |
| `/imu_node/imu/accel_gyro` | Imu | 200 Hz | IMU 센서 |
| `/odometry/filtered` | Odometry | 50 Hz | EKF 출력 (퓨전된 odom) |
| `/cmd_vel` | Twist | 20 Hz | MPPI controller 출력 |
