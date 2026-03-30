---
title: 5.4 Simulation Environment Setup
description: Ignition Gazebo simulation environment for AntBot
sidebar:
  order: 4
---

The `antbot_gazebo` package provides an Ignition Gazebo simulation environment for testing AntBot software without physical hardware.

## Prerequisites

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select antbot_gazebo
source install/setup.bash
```

## Launch

### Start Gazebo Simulation

```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

Wait ~15 seconds for the Gazebo window and controller spawners to complete.

Custom world:

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

### Manual Teleop

```bash
ros2 run antbot_teleop teleop_keyboard
```

### With Nav2 Navigation

To test autonomous navigation on top of the simulation, see [5.3 Nav2 Integration](/antbot/en/development-guide/navigation/).

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

## Simulation URDF

`antbot_gazebo/urdf/antbot_sim.xacro` reuses shared macros (base, wheel, sensors) from `antbot_description` and defines only the hardware interface and Gazebo plugins.

| Aspect | Real Robot | Simulation |
|--------|----------|-----------|
| ros2_control plugin | `BoardInterface` | `IgnitionSystem` |
| Command interfaces | velocity + acceleration | velocity only |
| Sensors | Physical drivers | Gazebo gpu_lidar + IMU |

## Simulated Sensors

| Sensor | Topic | Type | Rate |
|--------|-------|------|------|
| Front 2D LiDAR | `/scan_0` | `LaserScan` | 15 Hz |
| Back 2D LiDAR | `/scan_1` | `LaserScan` | 15 Hz |
| IMU | `/imu/data` | `Imu` | 100 Hz |
| Clock | `/clock` | `Clock` | — |

Topics are bridged from Ignition to ROS 2 via `ros_gz_bridge`.

## Controller Configuration

`swerve_controller_gazebo.yaml` is tuned for IgnitionSystem:

- `use_acceleration_command: false` — IgnitionSystem does not support acceleration interface
- `update_rate: 100` Hz — higher than real HW (20Hz) for simulation stability
- `enable_steering_scrub_compensator: false` — no real scrub friction in Gazebo
- `odom_integration_method: "analytic_swerve"` — exact for constant-velocity segments

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `worlds/empty.sdf` | Path to Gazebo world SDF file |
| `controller_delay` | `8.0` | Seconds to wait for Gazebo startup |

## Troubleshooting

### gpu_lidar returns only range_min

Check that the world SDF file uses `render_engine: ogre2`. Ignition gpu_lidar only works with ogre2.

### Controller spawn timeout

Increase the `controller_delay` argument:

```bash
ros2 launch antbot_gazebo gazebo.launch.py controller_delay:=15.0
```

### View URDF model only

To visualize the URDF without Gazebo:

```bash
ros2 launch antbot_description description.launch.py
```

For installation and build instructions, see [Software Environment Setup](/antbot/en/software/environment-setup/).
