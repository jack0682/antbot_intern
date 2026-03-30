# Nav2 Integration Debug Log — 2026-03-24

AntBot swerve-drive 로봇의 Gazebo 시뮬레이션 + Nav2 네비게이션 통합 과정에서 발생한 이슈와 해결 기록.

---

## 1. 빌드 의존성 누락

**증상**: `colcon build` 시 `controller_interface`, `asio`, `nmea_msgs`, `rtcm_msgs` 등 패키지를 찾지 못함.

**원인**: ROS 2 Humble 시스템 패키지 미설치.

**해결**:
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  libasio-dev ros-humble-nmea-msgs ros-humble-rtcm-msgs
rosdep install --from-paths src --ignore-src -y --skip-keys="ament_python"
```

---

## 2. antbot_camera 빌드 실패 (아키텍처 불일치)

**증상**: `librectrl.so: file in wrong format` 링크 에러.

**원인**: `antbot_camera/lib/librectrl.so`가 ARM용(Jetson) 바이너리인데 x86 시스템에서 빌드 시도.

**해결**: antbot_camera를 빌드에서 제외하고 `sim.launch.py`로 시뮬레이션 진행 (bringup.launch.py는 카메라 의존성으로 사용 불가).

---

## 3. Gazebo 라이다가 전부 range_min 반환

**증상**: `/scan_0`의 360개 레이가 모두 동일한 `range_min` 값으로 채워짐. RViz에서 로봇 주변에 원형으로 표시.

**원인**: `depot.sdf` 월드 파일의 Sensors 플러그인 `render_engine`이 **`ogre`**로 설정됨. Ignition Fortress의 `gpu_lidar`는 **`ogre2`** 렌더 엔진이 필요.

**해결**:
```xml
<!-- depot.sdf -->
<render_engine>ogre2</render_engine>  <!-- ogre → ogre2 -->
```

**교훈**: gpu_lidar가 모든 레이에서 range_min을 반환하면 센서 자체가 아닌 **렌더 엔진 설정**을 먼저 확인할 것.

---

## 4. Steering joint velocity NaN

**증상**: `/joint_states`에서 steering joint의 velocity가 `.nan`으로 보고됨.

**원인**: `ros2_control_sim.xacro`에서 steering joint에 `velocity` state_interface가 선언되지 않음. IgnitionSystem이 해당 인터페이스를 제공하지 않아 NaN 반환.

**해결**:
```xml
<!-- ros2_control_sim.xacro: 각 steering joint에 추가 -->
<state_interface name="velocity"/>
```

---

## 5. Swerve controller odom 미발행

**증상**: `/antbot_swerve_controller/odom` 토픽의 publisher count가 0. 컨트롤러는 active 상태.

**원인 (복합)**:

1. `enable_direct_joint_commands`가 기본값 `true` → `direct_joint_control()` 경로에서 early return → `calculate_odometry()` 미실행
2. 실제 odom 발행 토픽이 `/antbot_swerve_controller/odom`이 아닌 **`/odom`**

**해결**:
```yaml
# swerve_drive_controller_sim.yaml
enable_direct_joint_commands: false
```
```yaml
# ekf.yaml
odom0: /odom  # /antbot_swerve_controller/odom → /odom
```

---

## 6. odom→base_link TF 이중 발행으로 진동

**증상**: "Sensor origin out of map bounds" 경고 반복. TF가 두 위치 사이를 왔다 갔다.

**원인**: swerve controller(`enable_odom_tf: true`)와 EKF(`publish_tf: true`)가 **동시에** `odom→base_link` TF를 발행. navigation launch의 `ros2 param set`이 비동기라 적용 전까지 이중 발행.

**해결**: `disable_odom_tf` ExecuteProcess를 **3초 TimerAction으로 지연**하여 swerve controller가 완전히 활성화된 후 파라미터 변경.

```python
disable_odom_tf = TimerAction(
    period=3.0,
    actions=[ExecuteProcess(
        cmd=['ros2', 'param', 'set',
             '/antbot_swerve_controller', 'enable_odom_tf', 'false'],
        output='screen')])
```

---

## 7. Global planner 경로 생성 실패

**증상**: `GridBased: failed to create plan with tolerance 0.50` 반복.

**원인**: 로봇 위치의 global costmap 셀 cost가 100(lethal). 라이다 자기충돌 데이터(range_min 반환값)가 costmap에 장애물로 등록됨.

**해결**: costmap `obstacle_min_range`를 라이다의 유효 최소 거리보다 높게 설정.

```yaml
obstacle_min_range: 0.25   # 라이다 self-hit 데이터(~0.22m) 필터링
raytrace_min_range: 0.25
```

---

## 8. Costmap inflation 부족으로 벽 충돌

**증상**: 로봇이 벽에 근접해도 costmap에서 충분한 비용이 부과되지 않아 충돌.

**원인**: `inflation_radius: 0.35`가 로봇 inscribed radius(0.30m)와 거의 같아 inflation 여유 없음. global costmap은 `inflation_radius: 0.1`로 사실상 inflation 없음.

**해결**:
```yaml
inflation_radius: 0.75          # 0.35/0.1 → 0.75 (양쪽 costmap)
cost_scaling_factor: 1.5         # 3.0 → 1.5 (비용을 멀리까지 유지)
```

---

## 9. MPPI 컨트롤러 body heading 미정렬

**증상**: 로봇이 이동 방향과 무관하게 바퀴 중심으로 움직임 (crab-walking). 커브에서 swerve 방향 이상.

**원인**: MPPI `Omni` 모델이 (vx, vy, wz)를 자유롭게 조합. vy 제한이 느슨하고 heading 정렬 critic 가중치가 낮음.

**해결**:
```yaml
vy_max: 0.15                     # 0.5 → 0.15 (crab-walk 방지)
PreferForwardCritic: 15.0        # 5.0 → 15.0
PathAngleCritic: 20.0            # 2.0 → 20.0
TwirlingCritic: 15.0             # 10.0 → 15.0
```

---

## 10. 벽 충돌 시 EKF odom 폭주

**증상**: 벽에 부딪힌 후 odom이 급격히 드리프트, localization 완전 이탈.

**원인**: 바퀴 슬립으로 wheel velocity가 비정상 → EKF가 그대로 수용.

**해결**:
```yaml
# ekf.yaml
odom0_rejection_threshold: 2.0   # Mahalanobis distance 기반 outlier 거부
```

---

## 11. Swerve controller sim 특화 튜닝

**증상**: Gazebo의 완벽한 물리엔진에서 실제 하드웨어 가정(마찰, 지연)이 맞지 않음.

**적용한 sim 전용 파라미터**:

| 파라미터 | 기본값 | sim 값 | 이유 |
|----------|--------|--------|------|
| `non_coaxial_ik_iterations` | 0 | 2 | 55mm 오프셋 IK 보정 (Gazebo가 오차 노출) |
| `enable_steering_scrub_compensator` | true | false | Gazebo에 실제 scrub 없음 → 과보상 방지 |
| `velocity_rolling_window_size` | 1 | 5 | encoder 노이즈 평활화 |
| `odom_integration_method` | rk4 | analytic_swerve | piecewise-constant velocity에 정확 |

---

## 12. install 디렉토리 캐싱

**증상**: xacro 파일 수정 후에도 이전 설정으로 Gazebo 실행. scan_0가 여전히 360도, range_min=0.7.

**원인**: 이전에 `--symlink-install` 없이 빌드한 적이 있어 install에 **복사본**이 남아있음. 이후 수정이 반영되지 않음.

**해결**:
```bash
rm -rf install/antbot_description build/antbot_description
colcon build --symlink-install --packages-select antbot_description
```

**교훈**: `--symlink-install`을 처음부터 일관되게 사용하거나, 캐시 의심 시 build/install 디렉토리를 삭제 후 재빌드.

---

## 13. Progress checker 실패 (웨이포인트 네비게이션)

**증상**: `Failed to make progress` — 웨이포인트 순회 중 경로 재계획이 반복되면서 10초 내 0.5m를 이동하지 못해 abort.

**원인**: steering re-alignment에 시간이 소요되어 progress checker 기준 미달.

**해결**:
```yaml
progress_checker:
  required_movement_radius: 0.3   # 0.5 → 0.3 (작은 움직임도 인정)
  movement_time_allowance: 20.0   # 10.0 → 20.0 (re-alignment 시간 확보)
```

---

## 오픈소스 정리 작업

| 작업 | 내용 |
|------|------|
| 하드코딩 경로 제거 | `sim.launch.py`의 `/home/jack/...` → `os.path.join(bringup_dir, ...)` |
| 월드/맵 파일 이동 | `worlds/` → `antbot_bringup/worlds/`, `maps/` → `antbot_navigation/maps/` |
| 미사용 파일 삭제 | `base_sim.xacro`, `antbot_sim_display.xacro` |
| RViz config 생성 | `antbot_navigation/rviz/navigation.rviz` |
| README 작성 | `antbot_navigation/README.md` (EN), `README_ko.md` (KO) |
| Config 문서화 | EKF process noise 행렬, MPPI 튜닝 근거, SLAM 단위, footprint 동기화 주석 |
| Path Smoother 추가 | `SimpleSmoother` — 글로벌 경로 꺾임 완화 |
| flake8 수정 | `sim_nav.launch.py` 미사용 import 및 라인 길이 |
