# antbot_navigation

Nav2 navigation stack for AntBot holonomic swerve-drive robot.

Three navigation modes with separate configurations for simulation (`mode:=sim`) and real robot (`mode:=real`). Pre-tuned MPPI controller, AMCL localization, and SLAM Toolbox mapping.

## Prerequisites

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller

colcon build --symlink-install --packages-select antbot_navigation
source install/setup.bash
```

## Usage

Requires the robot (or Gazebo simulation) to be running first. See [antbot_gazebo](../antbot_gazebo/README.md) for simulation setup.

### SLAM (build map while navigating)

```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim    # or mode:=real
```

Save map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`

### Navigation (saved map + autonomous driving)

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=/path/to/map.yaml
```

### Localization Only (no planning)

```bash
ros2 launch antbot_navigation localization.launch.py mode:=sim map:=/path/to/map.yaml
```

## Sim / Real Mode

| Setting | `mode:=sim` | `mode:=real` |
|---------|-------------|--------------|
| Config directory | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 5.0 m/s | 2.0 m/s |
| Velocity smoother | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF noise | Low (ideal sensors) | Higher (real noise) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

## Package Structure

```
antbot_navigation/
├── config/
│   ├── sim/                  # Gazebo simulation configs
│   │   ├── nav2_params.yaml
│   │   ├── ekf.yaml
│   │   └── slam_toolbox_params.yaml
│   └── real/                 # Real robot configs
│       ├── nav2_params.yaml
│       ├── ekf.yaml
│       └── slam_toolbox_params.yaml
├── launch/
│   ├── slam.launch.py
│   ├── navigation.launch.py
│   └── localization.launch.py
├── maps/
│   └── depot_sim.yaml
└── rviz/
    └── navigation.rviz
```

## Sensor Topics

| Topic | Type | Source |
|-------|------|--------|
| `/scan_0` | `LaserScan` | Front 2D LiDAR |
| `/scan_1` | `LaserScan` | Back 2D LiDAR |
| `/odom` | `Odometry` | Swerve controller |
| `/imu_node/imu/accel_gyro` | `Imu` | IMU sensor |

## Troubleshooting

- **"Failed to create plan"** — Wrong initial pose. Use **2D Pose Estimate** in RViz.
- **Wall collision** — Increase `inflation_radius` or `collision_margin_distance` in nav2_params.yaml.
- **Odom drift after collision** — EKF `odom0_rejection_threshold` filters spikes. Re-localize if needed.
- **TF jitter** — Navigation launch auto-disables swerve controller's odom TF (EKF takes over).
