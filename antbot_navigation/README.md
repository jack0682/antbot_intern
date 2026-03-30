# antbot_navigation

Nav2 navigation stack for AntBot holonomic swerve-drive robot.

Provides three navigation modes (SLAM, localization, full navigation) with separate configurations for **simulation** and **real robot** environments. Pre-tuned for the MPPI controller, AMCL localization, and SLAM Toolbox mapping.

## Prerequisites

```bash
# Nav2 + dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

## Quick Start (Simulation)

### 1. Launch Gazebo simulation

```bash
ros2 launch antbot_bringup sim.launch.py
```

Wait for the Gazebo window to appear and the controller spawners to complete (~15 seconds).

### 2. Launch navigation with a pre-built map

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim \
  map:=$(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/maps/depot_sim.yaml
```

### 3. Open RViz for visualization

```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation)/share/antbot_navigation/rviz/navigation.rviz
```

### 4. Set initial pose and send goals

1. In RViz, click **2D Pose Estimate** and place the robot's approximate position on the map.
2. Click **Nav2 Goal** and click a destination. The robot will plan a path and follow it.

## Sim / Real Mode

All launch files accept a `mode` argument that selects the appropriate config directory and `use_sim_time` automatically:

```bash
# Simulation (default)
ros2 launch antbot_navigation slam.launch.py                   # mode:=sim implied
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=<map.yaml>

# Real robot
ros2 launch antbot_navigation slam.launch.py mode:=real
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=<map.yaml>
```

| Setting | `mode:=sim` | `mode:=real` |
|---------|-------------|--------------|
| Config directory | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 5.0 m/s | 2.0 m/s |
| Velocity smoother max | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF process noise | Low (ideal sensors) | Higher (real noise) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

## Navigation Modes

### Full Navigation (map + planning + control)

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=/path/to/map.yaml
```

### SLAM Mapping (build map while navigating)

```bash
ros2 launch antbot_navigation slam.launch.py mode:=real
```

Save the map when done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Localization Only (no planning)

```bash
ros2 launch antbot_navigation localization.launch.py mode:=real map:=/path/to/map.yaml
```

## Package Structure

```
antbot_navigation/
├── config/
│   ├── sim/                            # Gazebo simulation configs
│   │   ├── nav2_params.yaml
│   │   ├── ekf.yaml
│   │   └── slam_toolbox_params.yaml
│   └── real/                           # Real robot configs
│       ├── nav2_params.yaml
│       ├── ekf.yaml
│       └── slam_toolbox_params.yaml
├── launch/
│   ├── slam.launch.py                  # SLAM + Navigation
│   ├── localization.launch.py          # EKF + AMCL only
│   └── navigation.launch.py           # Full Nav2 stack
├── maps/
│   └── depot_sim.yaml
├── rviz/
│   └── navigation.rviz
└── docs/
```

## Configuration Files

| File | Description |
|------|-------------|
| `config/{mode}/nav2_params.yaml` | All Nav2 server parameters (MPPI, costmaps, AMCL, behavior) |
| `config/{mode}/ekf.yaml` | EKF sensor fusion (wheel odometry + IMU) |
| `config/{mode}/slam_toolbox_params.yaml` | SLAM Toolbox async mapping parameters |

Where `{mode}` is `sim` or `real`.

## Sensor Topics

| Topic | Type | Source |
|-------|------|--------|
| `/scan_0` | `sensor_msgs/LaserScan` | Front 2D LiDAR |
| `/scan_1` | `sensor_msgs/LaserScan` | Back 2D LiDAR |
| `/odom` | `nav_msgs/Odometry` | Swerve controller wheel odometry |
| `/imu_node/imu/accel_gyro` | `sensor_msgs/Imu` | IMU sensor |

These topic names are identical between simulation (via ros_gz_bridge) and real robot (via hardware drivers).

## Troubleshooting

### "Failed to create plan" error

- Robot is inside an obstacle on the costmap (wrong initial pose)
- Fix: Use **2D Pose Estimate** in RViz to correct the robot's position

### Robot collides with walls

- Increase `inflation_radius` in `nav2_params.yaml` (both local and global costmaps)
- Increase `ObstaclesCritic.collision_margin_distance` for MPPI

### Odometry drifts after collision

- The EKF's `odom0_rejection_threshold` filters large spikes
- If drift persists, re-localize with **2D Pose Estimate** in RViz

### "Sensor origin out of map bounds" warning

- Caused by dual TF publishers (both swerve controller and EKF)
- Navigation launch files handle this automatically by disabling the controller's TF
