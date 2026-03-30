---
title: 5.4 Simulation Environment Setup
description: Ignition Gazebo simulation environment setup guide for AntBot
sidebar:
  order: 4
---

AntBot can be operated and tested in Ignition Gazebo (Fortress) without physical hardware.
The simulation environment is provided by the `antbot_gazebo` package.

---

## Quick Start

### Prerequisites

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher
```

### Build and Run

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to antbot_gazebo
source install/setup.bash
```

**Terminal 1 — Gazebo Simulation**

```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

:::note
Wait ~8 seconds for the Gazebo window and controllers to load.
On slower GPUs, increase the delay:
```bash
ros2 launch antbot_gazebo gazebo.launch.py controller_delay:=15.0
```
:::

**Terminal 2 — Keyboard Teleop**

```bash
ros2 run antbot_teleop teleop_keyboard
```

For Nav2 autonomous navigation, see [5.3 Nav2 Integration](/antbot/en/development-guide/navigation/).

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `empty.sdf` | Path to Gazebo world SDF file |
| `controller_delay` | `8.0` | Seconds to wait for Gazebo startup |

---

## System Architecture

### Simulation Pipeline

```
User input (teleop / Nav2)
       │  /cmd_vel
       ▼
┌─ ROS 2 ───────────────────────────────────────┐
│  controller_manager (100Hz)                    │
│    ├── joint_state_broadcaster → /joint_states │
│    └── swerve_drive_controller                 │
│          ├── subscribes: /cmd_vel              │
│          ├── publishes:  /odom, /tf            │
│          └── command → IgnitionSystem          │
│  robot_state_publisher → /tf (base_link → *)  │
└────────────────────┬───────────────────────────┘
               ros_gz_bridge
┌────────────────────▼───────────────────────────┐
│  Ignition Gazebo (Fortress)                    │
│    ├── gpu_lidar (front) → /scan_0             │
│    ├── gpu_lidar (back)  → /scan_1             │
│    ├── IMU → /imu/data                         │
│    └── IgnitionSystem (ros2_control HW plugin) │
└────────────────────────────────────────────────┘
```

### Launch Sequence

```
gazebo.launch.py
  ├── Environment variables (IGN_GAZEBO_RESOURCE_PATH, PLUGIN_PATH)
  ├── xacro → URDF generation
  ├── ign gazebo -r world.sdf
  ├── Robot spawn (x=0, y=0, z=0.5)
  ├── robot_state_publisher
  ├── [controller_delay wait]
  ├── joint_state_broadcaster spawn
  ├── swerve_drive_controller spawn (after JSB completes)
  └── ros_gz_bridge (sensor topic bridging)
```

### Simulation vs Real Hardware

| Aspect | Real HW | Gazebo |
|--------|---------|--------|
| Hardware interface | BoardInterface | **IgnitionSystem** |
| Acceleration command | Supported | **Not supported** (velocity/position only) |
| 2D LiDAR | COIN D4 | **gpu_lidar** (ogre2 required) |
| IMU | antbot_imu node | **Gazebo IMU plugin** |
| Control rate | 20 Hz | **100 Hz** |
| Scrub compensation | Required | **Not needed** |
| IK iterations | 0 | **3** (55mm offset correction) |
| Odom integration | rk4 | **analytic_swerve** |
| Odom smoothing | window: 1 | **window: 10** |

---

## URDF Structure

### File Layout

```
antbot_sim.xacro (simulation entry point)
  ├── antbot_description/  (shared — same as real HW)
  │    ├── sensors.xacro, base.xacro, wheel.xacro
  └── antbot_gazebo/  (sim-specific)
       ├── ros2_control_gazebo.xacro   IgnitionSystem
       └── gazebo_plugins.xacro        friction + sensors
```

### Hardware Interface

```
IgnitionSystem (ign_ros2_control)
  ├── Wheels (x4):    velocity cmd → velocity/position state
  └── Steering (x4):  position cmd → position/velocity state
```

:::caution
IgnitionSystem does not support `acceleration` or `effort` command interfaces.
Controller config must set `use_acceleration_command: false`.
:::

### Friction Model

| Part | mu1 / mu2 | Description |
|------|-----------|-------------|
| base_link | 0.2 / 0.2 | Low friction (sliding) |
| wheel (x4) | 1.8 / 1.8 | High friction (traction) |
| steering (x4) | 0.0 / 0.0 | No friction (free rotation) |

### Sensors

| Sensor | Topic | Samples | Range | Noise |
|--------|-------|---------|-------|-------|
| Front 2D LiDAR | `/scan_0` | 720 | 0.6-20m | stddev 0.008 |
| Back 2D LiDAR | `/scan_1` | 720 | 0.6-20m | stddev 0.008 |
| IMU | `/imu/data` | — | — | angular 0.0003, linear 0.02 |

:::caution
gpu_lidar only works with the **ogre2** render engine.
Set `<render_engine>ogre2</render_engine>` in your world SDF.
:::

---

## Controller Configuration

Config file: `antbot_gazebo/config/swerve_controller_gazebo.yaml`

| Parameter | Sim value | HW value | Reason |
|-----------|-----------|----------|--------|
| `non_coaxial_ik_iterations` | **3** | 0 | Gazebo exposes 55mm offset error |
| `enable_steering_scrub_compensator` | **false** | true | No real scrub in Gazebo |
| `velocity_rolling_window_size` | **10** | 1 | Smooth sim encoder noise |
| `odom_integration_method` | **analytic_swerve** | rk4 | Exact for piecewise-constant |
| `use_acceleration_command` | **false** | true | IgnitionSystem limitation |

:::tip
Setting `non_coaxial_ik_iterations` to 0 in simulation causes odometry drift from the 55mm steering-wheel offset. Always set to 2-3 in simulation.
:::

---

## Custom Worlds

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

World SDF requirements:
- `render_engine: ogre2` (required for gpu_lidar)
- Physics, Sensors, UserCommands, SceneBroadcaster plugins
- ground_plane + sun (lighting)

---

## Troubleshooting

### gpu_lidar returns only range_min

Check that `render_engine` is `ogre2`. Using `ogre` causes all rays to return minimum values.

### Controller spawn timeout

Increase `controller_delay`: `controller_delay:=15.0`

### View URDF model only

```bash
ros2 launch antbot_description description.launch.py
```

For installation and build instructions, see [Software Environment Setup](/antbot/en/software/environment-setup/).
