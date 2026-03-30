# Troubleshooting

Nav2 네비게이션에서 발생하는 주요 문제와 해결 방법.

## 빌드/실행 문제

### "package not found" 에러

```
PackageNotFoundError: "package 'antbot_camera' not found"
```

`bringup.launch.py`는 카메라 의존성이 있음. 시뮬레이션은 `sim.launch.py`를 사용할 것.

### "Could not load mesh resource" (RViz)

```
Could not load mesh resource 'package://antbot_description/meshes/...'
```

`source install/setup.bash`를 먼저 실행해야 `package://` 경로를 resolve할 수 있음.

### Gazebo gpu_lidar가 전부 range_min 반환

월드 SDF 파일의 Sensors 플러그인 `render_engine`이 `ogre2`인지 확인:

```xml
<!-- WRONG -->
<render_engine>ogre</render_engine>

<!-- CORRECT -->
<render_engine>ogre2</render_engine>
```

Ignition gpu_lidar는 ogre2 렌더 엔진에서만 정상 동작.

---

## 네비게이션 문제

### "Failed to create plan"

**원인**: 로봇이 costmap에서 장애물 내부에 위치.

**해결**:
1. RViz에서 **2D Pose Estimate**로 로봇 위치 수정
2. 그래도 안 되면 costmap 초기화: `ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap`

### "Failed to make progress"

**원인**: 로봇이 `movement_time_allowance` 시간 내에 `required_movement_radius`만큼 이동하지 못함.

**해결**:
```yaml
# config/{mode}/nav2_params.yaml (mode = sim 또는 real)
progress_checker:
  required_movement_radius: 0.3   # 줄이기 (기본 0.5)
  movement_time_allowance: 20.0   # 늘리기 (기본 10.0)
```

### 로봇이 벽에 충돌

**해결 (순서대로 시도)**:
1. `inflation_radius` 증가 (0.75 → 1.0)
2. `cost_scaling_factor` 감소 (1.5 → 1.0)
3. MPPI `ObstaclesCritic.collision_margin_distance` 증가
4. `vx_max` 감소 (커브에서 과속 방지)

### 로봇이 crab-walking (옆으로 걸음)

**원인**: MPPI가 vy를 과도하게 사용.

**해결**: `vy_max`를 더 줄이거나 (0.15 → 0.05), `PreferForwardCritic` 가중치 증가.

### 경로가 지그재그

**원인**: Path Smoother 미적용 또는 NavFn 해상도 문제.

**해결**: `smoother_server`가 `lifecycle_manager_navigation`에 등록되어 있는지 확인.

---

## TF/Odometry 문제

### TF가 튀는 현상 (진동)

**원인**: swerve controller와 EKF가 동시에 `odom→base_link` TF를 발행.

**확인**:
```bash
ros2 param get /antbot_swerve_controller enable_odom_tf
```

`true`이면 수동으로 비활성화:
```bash
ros2 param set /antbot_swerve_controller enable_odom_tf false
```

navigation launch가 3초 지연 후 자동 비활성화하지만, 타이밍 문제로 실패할 수 있음.

### "Sensor origin out of map bounds"

위의 TF 이중 발행이 원인. 같은 해결법 적용.

### 충돌 후 odom 드리프트

EKF의 `odom0_rejection_threshold: 2.0`이 대부분의 스파이크를 필터링. 그래도 드리프트가 심하면:
1. RViz에서 **2D Pose Estimate**로 재로컬라이제이션
2. `velocity_rolling_window_size` 증가 (5 → 10)

### Steering joint velocity NaN

`ros2_control_sim.xacro`에서 steering joint에 `<state_interface name="velocity"/>` 확인.

---

## 센서 문제

### /scan_0 데이터 없음

```bash
# Ignition 토픽 확인
ign topic -l | grep scan

# ROS 브릿지 확인
ros2 topic hz /scan_0
```

### /lidar_3d_points 데이터 없음

Ignition은 `/lidar_3d_points/points`로 발행. bridge에서 리매핑 필요:

```python
# sim.launch.py bridge arguments
'/lidar_3d_points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
# remapping
('/lidar_3d_points/points', '/lidar_3d_points')
```

### 라이다 데이터가 원형으로만 보임 (장애물 없음)

1. `render_engine`이 `ogre2`인지 확인 (가장 흔한 원인)
2. 라이다 `<min>`이 너무 높지 않은지 확인
3. 월드에 실제 벽/장애물 메시가 있는지 확인

---

## 진단 명령어 모음

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
ros2 param get /ekf_filter_node use_sim_time   # mode:=sim이면 true, mode:=real이면 false

# Costmap 확인
ros2 topic echo /local_costmap/costmap --once | head -20

# Nav2 lifecycle 상태
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
```
