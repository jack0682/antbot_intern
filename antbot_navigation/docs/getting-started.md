# Getting Started

시뮬레이션에서 AntBot Nav2 네비게이션을 5분 안에 실행하는 가이드.

## 의존성 설치

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller ros-humble-ros-gz \
  ros-humble-ign-ros2-control ros-humble-xacro
```

## 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 실행 (3개 터미널)

### Terminal 1 — Gazebo 시뮬레이션

```bash
source install/setup.bash
ros2 launch antbot_bringup sim.launch.py
```

Gazebo 창이 나타나고 컨트롤러가 로드될 때까지 약 15초 대기.

### Terminal 2 — Nav2 네비게이션

```bash
source install/setup.bash
ros2 launch antbot_navigation navigation.launch.py \
  map:=$(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/maps/depot_sim.yaml \
  mode:=sim
```

### Terminal 3 — RViz 시각화

```bash
source install/setup.bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

## 로봇 이동시키기

### 단일 목표

1. RViz에서 **2D Pose Estimate**로 초기 위치 설정
2. **Nav2 Goal** 클릭 후 목적지 클릭

### 웨이포인트 순회

1. RViz 메뉴 → **Panels** → **Add New Panel** → `nav2_rviz_plugins/Navigation2`
2. Nav2 패널에서 **Waypoint / Nav Through Poses Mode** 체크
3. **Nav2 Goal**로 여러 지점을 순서대로 클릭
4. Nav2 패널의 **Start Waypoint Following** 클릭

### CLI로 이동

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

## 다른 네비게이션 모드

### SLAM 맵핑 (맵 생성하면서 네비게이션)

```bash
# Terminal 2 대신:
ros2 launch antbot_navigation slam.launch.py mode:=sim
```

맵 저장: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`

### 로컬라이제이션 전용 (경로계획 없음)

```bash
ros2 launch antbot_navigation localization.launch.py \
  map:=/path/to/map.yaml mode:=sim
```
