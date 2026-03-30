---
title: 5.3 Nav2 Integration
description: AntBot Nav2 Navigation Guide
sidebar:
  order: 3
---

The `antbot_navigation` package provides [Nav2](https://docs.nav2.org/) navigation stack integration for the AntBot swerve-drive robot.

## Overview

Nav2 is the official ROS 2 navigation framework for autonomous robot movement with obstacle avoidance. AntBot uses MPPI controller, OmniMotionModel AMCL, and EKF sensor fusion, tuned for holonomic swerve-drive characteristics.

## Prerequisites

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select antbot_navigation
source install/setup.bash
```

## Navigation Modes

### SLAM (Map Building + Navigation)

Start without a map and build one while exploring the environment.

```bash
# On Gazebo simulation
ros2 launch antbot_navigation slam.launch.py mode:=sim

# On real robot
ros2 launch antbot_navigation slam.launch.py mode:=real
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Navigation (Saved Map + Autonomous Driving)

Full autonomous navigation with a saved map and AMCL localization.

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim \
  map:=/path/to/map.yaml
```

### Localization Only (No Planning)

Test localization before enabling autonomous navigation.

```bash
ros2 launch antbot_navigation localization.launch.py mode:=sim \
  map:=/path/to/map.yaml
```

## Sim / Real Mode

All launch files accept a `mode` argument that auto-selects the config directory and `use_sim_time`:

| Setting | `mode:=sim` | `mode:=real` |
|---------|-------------|--------------|
| Config directory | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 5.0 m/s | 2.0 m/s |
| Velocity smoother | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF process noise | Low (ideal sensors) | Higher (real noise) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

## Quick Start (Simulation)

For simulation environment setup, see [5.4 Simulation Environment Setup](/antbot/en/development-guide/simulation/).

**Terminal 1** — Gazebo:

```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

**Terminal 2** — SLAM navigation:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim
```

**Terminal 3** — RViz:

```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

In RViz:
1. Click **2D Pose Estimate** to set initial position
2. Click **Nav2 Goal** to send a destination

## Architecture

```
         /cmd_vel (Twist)
              ▲
              │
┌─────────────┴──────────────────────────┐
│  Nav2 Stack                            │
│  BT Navigator → Planner → Controller  │
│  (A* path)      (MPPI Omni)           │
│        ↕            ↕                  │
│  Global Costmap  Local Costmap         │
│  (static+obs)    (obs+inflation)       │
│        ↕                               │
│  AMCL / SLAM Toolbox                   │
│  (map → odom TF)                       │
└────────────────────────────────────────┘
              │
              ▼
┌────────────────────────────────────────┐
│  EKF (robot_localization)              │
│  Input: /odom (vx,vy) + IMU (vyaw)    │
│  Output: odom → base_link TF (50Hz)   │
└────────────────────────────────────────┘
              │
              ▼
   Swerve Controller (IK → motor commands)
```

### TF Tree

```
map → odom → base_link → [sensor frames]
 ^       ^
 │       │
AMCL    EKF (wheel odom + IMU fusion)
```

Navigation launch files disable the swerve controller's `odom→base_link` TF and let the EKF publish it instead via sensor fusion.

## Swerve-Drive Specific Configuration

### MPPI Controller

MPPI is chosen over DWB because its rollout-based optimization handles steering re-alignment delays and discontinuous mode transitions better.

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `motion_model` | `Omni` | Holonomic 3-DOF (vx, vy, wz) |
| `vy_max` | 1.0 (sim) / 0.5 (real) | Limit lateral motion (prevent crab-walking) |
| `PreferForwardCritic` | weight 5.0 | Prefer forward-facing travel |
| `TwirlingCritic` | weight 5.0 | Suppress unnecessary spinning |

### EKF Sensor Fusion

| Source | Fused States |
|--------|-------------|
| Wheel odom (`/odom`) | vx, vy, vyaw |
| IMU | yaw, vyaw (differential mode) |

### Dual LiDAR Costmap

- Front LiDAR (`/scan_0`): AMCL/SLAM input + costmap
- Back LiDAR (`/scan_1`): costmap only (reverse safety)

### AMCL

Uses `OmniMotionModel` — essential for swerve robots to model lateral motion in localization. The default `DifferentialMotionModel` significantly degrades accuracy.

## Sensor Topics

| Topic | Type | Source |
|-------|------|--------|
| `/scan_0` | `LaserScan` | Front 2D LiDAR |
| `/scan_1` | `LaserScan` | Back 2D LiDAR |
| `/odom` | `Odometry` | Swerve controller |
| `/imu_node/imu/accel_gyro` | `Imu` | IMU sensor |

Topic names are identical between simulation (ros_gz_bridge) and real robot (hardware drivers).

## Troubleshooting

- **"Failed to create plan"** — Wrong initial pose. Use **2D Pose Estimate** in RViz.
- **Wall collision** — Increase `inflation_radius` or `collision_margin_distance`.
- **Odom drift after collision** — EKF `odom0_rejection_threshold` filters spikes. Re-localize if persistent.
- **TF jitter** — Navigation launch auto-disables swerve controller TF (EKF takes over).

For `/cmd_vel` and `/odom` topic specifications, see [Key ROS Topics/Services](/antbot/en/development-guide/ros-topics/).
