# antbot_gazebo

Ignition Gazebo simulation package for AntBot swerve-drive robot.

## Prerequisites

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to antbot_gazebo antbot_teleop
source install/setup.bash
```

## Quick Start

**Terminal 1** — Launch Gazebo simulation:
```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

**Terminal 2** — Keyboard teleop:
```bash
ros2 run antbot_teleop teleop_keyboard
```

Specify a world by name (resolved via `config/worlds.yaml`) or full path:
```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

## Package Structure

```
antbot_gazebo/
├── config/
│   ├── swerve_controller_gazebo.yaml   # Sim-specific controller params
│   └── worlds.yaml                     # World name → SDF path mapping
├── launch/
│   └── gazebo.launch.py                # Gazebo + robot spawn + controllers
├── urdf/
│   ├── antbot_sim.xacro                # Top-level sim URDF
│   ├── gazebo_plugins.xacro            # Sensor plugins (LiDAR, IMU)
│   └── ros2_control_gazebo.xacro       # IgnitionSystem hardware interface
└── worlds/
    ├── empty.sdf                       # Empty world (default)
    └── depot.sdf                       # Warehouse-style world
```

## Simulated Sensors

| Sensor | Topic | Type | Rate |
|--------|-------|------|------|
| Front 2D LiDAR | `/scan_0` | `LaserScan` | 10 Hz |
| Back 2D LiDAR | `/scan_1` | `LaserScan` | 10 Hz |
| IMU | `/imu/data` | `Imu` | 200 Hz |
| Clock | `/clock` | `Clock` | — |

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `empty` | World name (from `worlds.yaml`) or full path to SDF file |
