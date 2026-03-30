# Simulation Environment

Gazebo Ignition Fortress 기반 AntBot 시뮬레이션 환경 설명.

## 구성 요소

### 월드 파일

`antbot_bringup/worlds/depot.sdf` — 창고형 시뮬레이션 환경.

| 항목 | 값 |
|------|-----|
| 렌더 엔진 | **ogre2** (gpu_lidar 필수) |
| 물리 스텝 | 2ms |
| 크기 | 30.16m x 15.20m |

### 시뮬레이션 URDF

`antbot_description/urdf/antbot_sim.xacro` — 실제 하드웨어 URDF의 sim 변형.

차이점:
- `ros2_control_sim.xacro` — `IgnitionSystem` 사용 (BoardInterface 대신)
- `gazebo.xacro` — 센서 플러그인 (gpu_lidar, IMU)
- 동일 메시, 동일 관성, 동일 센서 위치

### 센서 시뮬레이션

| 센서 | 타입 | 스펙 | 토픽 |
|------|------|------|------|
| 전방 2D LiDAR | gpu_lidar | 360 samples, 0.1-20m, 10Hz | `/scan_0` |
| 후방 2D LiDAR | gpu_lidar | 360 samples, 0.1-20m, 10Hz | `/scan_1` |
| 3D LiDAR | gpu_lidar (3D) | 720x16, 0.05-70m, 10Hz | `/lidar_3d_points` |
| IMU | imu | 200Hz, gaussian noise | `/imu` → `/imu_node/imu/accel_gyro` |

### ros_gz_bridge 토픽 매핑

```
Ignition → ROS 2:
  /scan_0                    → /scan_0 (LaserScan)
  /scan_1                    → /scan_1 (LaserScan)
  /lidar_3d_points/points    → /lidar_3d_points (PointCloud2)
  /imu                       → /imu_node/imu/accel_gyro (Imu)
  /clock                     → /clock (Clock)
```

## ros2_control in Simulation

### Hardware Interface

실제 하드웨어: `antbot/hw_interface/BoardInterface` (Dynamixel USB serial)
시뮬레이션: `ign_ros2_control/IgnitionSystem` (Gazebo physics)

### Command/State Interfaces

```yaml
Wheel joints:
  command: velocity
  state: velocity, position

Steering joints:
  command: position
  state: position, velocity    # velocity 필수 (NaN 방지)
```

### Sim 전용 컨트롤러 파라미터

`antbot_bringup/config/swerve_drive_controller_sim.yaml`:

```yaml
update_rate: 50               # Hz (실제 HW는 20Hz)
non_coaxial_ik_iterations: 2  # 55mm 오프셋 IK 보정
enable_steering_scrub_compensator: false  # Gazebo에 scrub 없음
velocity_rolling_window_size: 5           # odom 노이즈 평활화
odom_integration_method: "analytic_swerve" # 정확한 원호 적분
enable_odom_tf: true          # standalone sim용 (Nav2 시 비활성화)
```

## Launch 순서

```
sim.launch.py 실행 시:

t=0s    Ignition Gazebo 시작, 월드 로드
t=0s    robot_state_publisher 시작 (URDF → /robot_description, /tf)
t=0s    spawn_robot (URDF → Gazebo 모델)
t=0s    ros_gz_bridge 시작 (센서 토픽 브릿지)
t=10s   joint_state_broadcaster 스포너 (TimerAction 대기)
t=~12s  JSB 완료 → antbot_swerve_controller 스포너 (OnProcessExit)
t=~15s  컨트롤러 활성화 완료, /cmd_vel 구독 시작
```

## 커스텀 월드 사용

```bash
ros2 launch antbot_bringup sim.launch.py world:=/path/to/custom.sdf
```

SDF 파일 요구사항:
- `render_engine`이 **`ogre2`** 이어야 gpu_lidar 동작
- Physics, Sensors, UserCommands, SceneBroadcaster 플러그인 필요

## 알려진 제한 사항

1. **body mesh self-hit**: 2D LiDAR가 본체 내부에 위치하여 일부 레이가 body mesh에 맞음. costmap `obstacle_min_range: 0.25`로 필터링.
2. **steering re-alignment 지연 없음**: Gazebo 물리가 즉각 반응하여 실제 하드웨어보다 빠름.
3. **antbot_camera 미지원**: `librectrl.so`가 ARM 전용이라 x86에서 빌드 불가. sim에서 카메라 없이 동작.
