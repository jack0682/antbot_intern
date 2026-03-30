# antbot_navigation

AntBot 홀로노믹 스워브 드라이브 로봇을 위한 Nav2 네비게이션 스택.

MPPI 컨트롤러, AMCL 로컬라이제이션, SLAM Toolbox 맵핑을 위한 사전 튜닝된 설정을 제공하며, **시뮬레이션**과 **실제 로봇** 환경별 별도 설정을 지원합니다.

## 사전 준비

```bash
# Nav2 및 의존성 설치
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller

# 워크스페이스 빌드
colcon build --symlink-install
source install/setup.bash
```

## 빠른 시작 (시뮬레이션)

### 1. Gazebo 시뮬레이션 실행

```bash
ros2 launch antbot_bringup sim.launch.py
```

Gazebo 창이 나타나고 컨트롤러 스포너가 완료될 때까지 약 15초 대기합니다.

### 2. 저장된 맵으로 네비게이션 실행

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim \
  map:=$(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/maps/depot_sim.yaml
```

### 3. RViz 시각화 열기

```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

### 4. 초기 위치 설정 및 목표점 전송

1. RViz에서 **2D Pose Estimate** 클릭 후 맵 위에 로봇 위치를 지정합니다.
2. **Nav2 Goal** 클릭 후 목적지를 클릭하면 로봇이 경로를 계획하고 추종합니다.

## Sim / Real 모드

모든 launch 파일은 `mode` 인자를 받아 설정 디렉토리와 `use_sim_time`을 자동 선택합니다:

```bash
# 시뮬레이션 (기본값)
ros2 launch antbot_navigation slam.launch.py                    # mode:=sim 생략 가능
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=<맵.yaml>

# 실제 로봇
ros2 launch antbot_navigation slam.launch.py mode:=real
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=<맵.yaml>
```

| 설정 | `mode:=sim` | `mode:=real` |
|------|-------------|--------------|
| 설정 디렉토리 | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 5.0 m/s | 2.0 m/s |
| 속도 스무더 최대값 | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF 프로세스 노이즈 | 낮음 (이상적 센서) | 높음 (실제 노이즈) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

## 네비게이션 모드

### 전체 네비게이션

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=/경로/맵파일.yaml
```

### SLAM 맵핑

```bash
ros2 launch antbot_navigation slam.launch.py mode:=real
```

맵 저장: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`

### 로컬라이제이션 전용

```bash
ros2 launch antbot_navigation localization.launch.py mode:=real map:=/경로/맵파일.yaml
```

## 패키지 구조

```
antbot_navigation/
├── config/
│   ├── sim/                            # Gazebo 시뮬레이션용 설정
│   │   ├── nav2_params.yaml
│   │   ├── ekf.yaml
│   │   └── slam_toolbox_params.yaml
│   └── real/                           # 실제 로봇용 설정
│       ├── nav2_params.yaml
│       ├── ekf.yaml
│       └── slam_toolbox_params.yaml
├── launch/
│   ├── slam.launch.py                  # SLAM + Navigation
│   ├── localization.launch.py          # EKF + AMCL 전용
│   └── navigation.launch.py           # 전체 Nav2 스택
├── maps/
│   └── depot_sim.yaml
└── docs/
```

## 설정 파일

| 파일 | 설명 |
|------|------|
| `config/{mode}/nav2_params.yaml` | Nav2 전체 서버 파라미터 (MPPI, 코스트맵, AMCL, 행동 서버) |
| `config/{mode}/ekf.yaml` | EKF 센서 퓨전 (바퀴 오도메트리 + IMU) |
| `config/{mode}/slam_toolbox_params.yaml` | SLAM Toolbox 비동기 맵핑 파라미터 |

`{mode}`는 `sim` 또는 `real`입니다.

## 센서 토픽

| 토픽 | 타입 | 소스 |
|------|------|------|
| `/scan_0` | `sensor_msgs/LaserScan` | 전방 2D 라이다 |
| `/scan_1` | `sensor_msgs/LaserScan` | 후방 2D 라이다 |
| `/odom` | `nav_msgs/Odometry` | 스워브 컨트롤러 바퀴 오도메트리 |
| `/imu_node/imu/accel_gyro` | `sensor_msgs/Imu` | IMU 센서 |

시뮬레이션(ros_gz_bridge)과 실제 로봇(물리 드라이버) 모두 동일한 토픽명을 사용합니다.

## 문제 해결

### "Failed to create plan" 에러

- 코스트맵에서 로봇이 장애물 내부에 위치 (초기 위치 오류)
- 해결: RViz에서 **2D Pose Estimate**로 로봇 위치를 수정

### 로봇이 벽에 충돌

- `nav2_params.yaml`에서 `inflation_radius` 증가
- MPPI의 `ObstaclesCritic.collision_margin_distance` 증가

### 충돌 후 오도메트리 드리프트

- EKF의 `odom0_rejection_threshold`가 스파이크를 필터링
- 드리프트 지속 시 RViz에서 **2D Pose Estimate**로 재로컬라이제이션

### "Sensor origin out of map bounds" 경고

- 스워브 컨트롤러와 EKF가 동시에 TF를 발행할 때 발생
- 네비게이션 launch가 자동으로 컨트롤러 TF를 비활성화
