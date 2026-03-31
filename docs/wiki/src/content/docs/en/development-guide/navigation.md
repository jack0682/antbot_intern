---
title: 5.3 Nav2 Integration
description: AntBot Nav2 Navigation Stack Integration Guide
sidebar:
  order: 3
---

AntBot uses Nav2, the official ROS 2 navigation framework, for autonomous driving.
Custom tuning is applied for the 4-wheel independent swerve-drive characteristics.

---

## Overview

### Nav2 Pipeline

![Nav2 Pipeline](../../../../assets/images/nav2_pipeline_en.png)

### AntBot vs Standard diff-drive

| Aspect | Standard diff-drive | AntBot swerve |
|--------|---------------------|---------------|
| Controller | DWB | **MPPI** (rollout-based optimization) |
| Motion Model (AMCL) | DifferentialMotionModel | **OmniMotionModel** |
| Velocity DOF | vx, wz (2DOF) | **vx, vy, wz (3DOF)** |
| Costmap Inflation | ~0.3m | **0.75m** (steering overshoot margin) |
| LiDAR | Single | **Dual 2D** (front + back) |
| Odometry TF | Direct publish | **EKF sensor fusion** (collision protection) |

---

## Quick Start

### Prerequisites

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller
```

### Run (3 Terminals)

**Terminal 1 — Gazebo Simulation**

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
```

:::note
Wait ~8-15 seconds for the Gazebo window and controller spawners to complete.
The `world` argument accepts a world name registered in `worlds.yaml` or a full path to an SDF file.
:::

**Terminal 2 — Nav2 Navigation**

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim world:=depot
```

:::tip
Using `world:=depot` automatically resolves the corresponding map file (`depot_sim.yaml`) from `worlds.yaml`. To specify a map path directly, use the `map:=` argument instead:
```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim \
  map:=/path/to/my_map.yaml
```
:::

**Terminal 3 — RViz**

```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation --share)/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

### Controlling the Robot

**Single goal**: In RViz, click **2D Pose Estimate** to set initial pose → click **Nav2 Goal**

**Waypoint following**: RViz → Panels → Add New Panel → `nav2_rviz_plugins/Navigation2` → check Waypoint Mode → click multiple goals → Start Waypoint Following

**CLI**:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"
```

### Navigation Modes

| Launch file | Purpose | Features |
|-------------|---------|----------|
| `navigation.launch.py` | Autonomous driving with saved map | AMCL + full Nav2 stack |
| `slam.launch.py` | Build map while navigating | SLAM Toolbox (no map needed) |
| `localization.launch.py` | Localization only | No planning, position estimation only |

Save map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`

---

## World and Map Management

### Directory Structure

World (SDF) and map (PGM/YAML) files are co-located in `antbot_navigation/maps/`:

```
antbot_navigation/maps/
├── worlds.yaml          # World name → SDF/map path mapping
├── depot.sdf            # Gazebo world file
├── depot_sim.pgm        # 2D occupancy grid map
└── depot_sim.yaml       # Map metadata (resolution, origin, etc.)
```

### worlds.yaml Configuration

`worlds.yaml` maps world names to their SDF and Nav2 map files:

```yaml
worlds:
  depot:
    sdf: depot.sdf           # Gazebo world SDF
    map: depot_sim.yaml       # Nav2 map YAML
```

When launching with `world:=depot`:
- **Gazebo**: loads `maps/depot.sdf`
- **Navigation**: passes `maps/depot_sim.yaml` to the map server

### Adding a New World

1. **Create a map with SLAM** (or import externally):

   ```bash
   # Start SLAM
   ros2 launch antbot_navigation slam.launch.py mode:=sim
   # Save map
   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_world
   ```

2. **Place files**: copy `my_world.pgm`, `my_world.yaml`, and `my_world.sdf` into `antbot_navigation/maps/`

3. **Register in worlds.yaml**:

   ```yaml
   worlds:
     depot:
       sdf: depot.sdf
       map: depot_sim.yaml
     my_world:                    # New world
       sdf: my_world.sdf
       map: my_world.yaml
   ```

4. **Launch**:

   ```bash
   ros2 launch antbot_gazebo gazebo.launch.py world:=my_world
   ros2 launch antbot_navigation navigation.launch.py mode:=sim world:=my_world
   ```

:::note
New worlds can be added by editing `worlds.yaml` alone — no launch file changes needed.
The `map` argument can still be used to override with any arbitrary map file path.
:::

---

## Sim / Real Mode

All launch files accept a `mode` argument that auto-selects the config directory and `use_sim_time`:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim    # simulation
ros2 launch antbot_navigation slam.launch.py mode:=real   # real robot
```

| Setting | `mode:=sim` | `mode:=real` |
|---------|-------------|--------------|
| Config directory | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 1.0 m/s | 2.0 m/s |
| Velocity smoother | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF IMU topic | `/imu/data` | `/imu_node/imu/accel_gyro` |
| EKF process noise | Low (ideal sensors) | Higher (real noise) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

---

## System Architecture

### TF Tree

![TF Tree](../../../../assets/images/tf_tree.png)

| Transform | Publisher |
|-----------|----------|
| `map → odom` | AMCL or SLAM Toolbox |
| `odom → base_link` | EKF (Nav2 mode) or swerve controller (standalone) |
| `base_link → *` | robot_state_publisher |

### odom TF Handover

:::caution
If both swerve controller and EKF publish `odom→base_link` TF simultaneously, jitter occurs.
Navigation launch auto-disables the controller's TF publish,
and Nav2 nodes start with an 8-second delay to allow EKF to establish the TF first.
If you see jitter, disable manually:

```bash
ros2 param set /antbot_swerve_controller enable_odom_tf false
```
:::

---

## Configuration

Config files: `antbot_navigation/config/{sim,real}/`

### MPPI Controller

AntBot's steering limit is **+/-60 degrees**, making pure crab motion physically impossible.
Direction changes use a **rotate-in-place then drive-forward** pattern, and MPPI parameters are tuned accordingly.

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Omni"
  time_steps: 56           # 2.8s lookahead
  batch_size: 2000
  vx_max: 1.0              # sim (real: 2.0)
  vy_max: 0.1              # nearly disabled (±60deg steering limit)
```

#### MPPI Critics

| Critic | Weight | Purpose |
|--------|--------|---------|
| ConstraintCritic | 4.0 | Velocity constraint violation penalty |
| GoalCritic | 5.0 | Goal approach reward |
| **PreferForwardCritic** | **15.0** | **Strongly align heading with travel direction (swerve key)** |
| **PathAngleCritic** | **15.0** | **Body angle = path direction (swerve key)** |
| **TwirlingCritic** | **10.0** | **Suppress unnecessary spinning (swerve key)** |
| ObstaclesCritic | — | Collision avoidance |
| PathAlignCritic | 10.0 | Trajectory-path alignment |
| PathFollowCritic | 5.0 | Global path following |

:::note[Steering Limit and MPPI Tuning]
`vy_max` is limited to 0.1 and `PreferForwardCritic` / `PathAngleCritic` weights are high,
forcing MPPI to **rotate in place to align heading first, then drive forward** rather than crab-walking.
Increasing `vy_max` causes the steering to hit its limits, producing unnatural motion.
:::

### AMCL

```yaml
amcl:
  robot_model_type: "nav2_amcl::OmniMotionModel"  # required for holonomic
  scan_topic: /scan_0
```

:::caution
`OmniMotionModel` is **required** for swerve robots. The default `DifferentialMotionModel` cannot model lateral motion (vy), significantly degrading localization accuracy.
:::

### EKF Sensor Fusion

```yaml
odom0: /odom                    # vx, vy, vyaw
imu0: /imu/data                 # yaw, vyaw (differential mode) — sim
# imu0: /imu_node/imu/accel_gyro  # real robot
odom0_rejection_threshold: 2.0  # reject collision spikes
```

:::note[Collision Protection]
When wheel slip occurs during wall collisions, velocity spikes are automatically rejected by `odom0_rejection_threshold`, preventing odom drift.
:::

:::caution[Sim vs Real IMU Topic]
In simulation, Gazebo ros_gz_bridge publishes IMU on `/imu/data`, while the real robot uses `/imu_node/imu/accel_gyro`. The correct topic is configured in `config/sim/ekf.yaml` and `config/real/ekf.yaml` respectively.
:::

### Costmap

Robot footprint 0.70m x 0.60m, dual LiDAR (`/scan_0` front + `/scan_1` back).

| Aspect | Local Costmap | Global Costmap |
|--------|---------------|----------------|
| Frame | `odom` | `map` |
| Size | 5m x 5m rolling | Full map |
| Update | 5Hz | 1Hz |
| Layers | obstacle x2 + inflation | static + obstacle x2 + inflation |

---

## Troubleshooting

### "Failed to create plan"

Robot is inside an obstacle on the costmap. Use **2D Pose Estimate** in RViz, or:

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### Wall collision

1. Increase `inflation_radius` (0.75 → 1.0)
2. Decrease `cost_scaling_factor` (1.5 → 1.0)
3. Increase `ObstaclesCritic.collision_margin_distance`

### Crab-walking

Reduce `vy_max` or increase `PreferForwardCritic` weight.
Since the steering limit is +/-60 degrees, `vy_max: 0.1` or lower is recommended.

### Missing map frame / TF timeout

If Gazebo is restarted, sim time resets to 0 and all TF buffers are invalidated.
**Always restart Gazebo and Navigation together.**

### Diagnostic Commands

```bash
ros2 topic hz /scan_0                    # LiDAR check
ros2 topic hz /odometry/filtered         # EKF output check
ros2 run tf2_ros tf2_echo map odom       # TF check
ros2 param get /antbot_swerve_controller enable_odom_tf
ros2 lifecycle get /controller_server    # Nav2 state
ros2 control list_controllers            # Controller state
```

:::tip
For `/cmd_vel` and `/odom` topic specifications, see [5.2 Key ROS Topics/Services](/antbot/en/development-guide/ros-topics/).
For simulation environment details, see [5.4 Simulation Environment Setup](/antbot/en/development-guide/simulation/).
:::
