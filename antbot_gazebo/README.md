# antbot_gazebo

Ignition Gazebo simulation package for AntBot swerve-drive robot.

## Prerequisites

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher

# For navigation in simulation, also install:
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select antbot_gazebo antbot_navigation
source install/setup.bash
```

## Quick Start

### Gazebo Only (Manual Teleop)

**Terminal 1** — Launch Gazebo simulation:
```bash
ros2 launch antbot_gazebo gazebo.launch.py
```
Wait ~15 seconds for Gazebo window + controller spawners to complete.

**Terminal 2** — Keyboard teleop:
```bash
ros2 run antbot_teleop teleop_keyboard
```

Custom world:
```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

### Gazebo + SLAM Navigation

**Terminal 1** — Gazebo:
```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

**Terminal 2** — SLAM + Nav2:
```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim
```

**Terminal 3** — RViz:
```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

Drive around with teleop or use **Nav2 Goal** in RViz to build a map.

Save the map when done:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Gazebo + Map Navigation

**Terminal 1** — Gazebo:
```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

**Terminal 2** — Navigation with saved map:
```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim \
  map:=$(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/maps/depot_sim.yaml
```

**Terminal 3** — RViz:
```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

In RViz:
1. Click **2D Pose Estimate** to set initial position on the map
2. Click **Nav2 Goal** to send a destination

## Package Structure

```
antbot_gazebo/
├── config/
│   └── swerve_controller_gazebo.yaml   # Sim-specific controller params
├── launch/
│   └── gazebo.launch.py                # Gazebo + robot spawn + controllers
├── urdf/
│   ├── antbot_sim.xacro                # Top-level sim URDF
│   ├── gazebo_plugins.xacro            # Sensor plugins (LiDAR, IMU)
│   └── ros2_control_gazebo.xacro       # IgnitionSystem hardware interface
└── worlds/
    ├── empty.sdf                       # Empty world
    └── depot.sdf                       # Warehouse-style world
```

## Simulated Sensors

| Sensor | Topic | Type | Rate |
|--------|-------|------|------|
| Front 2D LiDAR | `/scan_0` | `LaserScan` | 10 Hz |
| Back 2D LiDAR | `/scan_1` | `LaserScan` | 10 Hz |
| IMU | `/imu/data` | `Imu` | 200 Hz |
| Clock | `/clock` | `Clock` | — |

Topics are bridged via `ros_gz_bridge` to match real robot topic names.

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `worlds/empty.sdf` | Path to SDF world file |
| `controller_delay` | `8.0` | Seconds to wait for Gazebo before spawning controllers |
