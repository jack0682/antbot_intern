# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AntBot is a ROS 2 Humble robot stack for a 4-wheel independent swerve-drive autonomous delivery platform by ROBOTIS AI. It runs on Ubuntu 22.04 / Jetson Orin with C++17 and Python 3.

## Build Commands

```bash
# Full workspace build (from workspace root, e.g. ~/antbot_ws or /home/ros2_ws)
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select antbot_swerve_controller

# Build a package and its dependencies
colcon build --symlink-install --packages-up-to antbot_hw_interface

# Source the workspace after building
source install/setup.bash

# Install dependencies (run once from the antbot source directory)
cd src/antbot && bash setting.sh
```

## Linting

CI runs 8 linters via GitHub Actions (`.github/workflows/ros-lint.yml`): `cppcheck`, `cpplint`, `uncrustify`, `flake8`, `pep257`, `lint_cmake`, `xmllint`, `copyright`. The `vanjee_lidar_sdk` and `vanjee_lidar_msg` packages are excluded from linting.

All C++ packages use `-Wall -Wextra -Wpedantic`. There is no `.clang-format` or `.clang-tidy` — linting relies on ament_lint defaults.

```bash
# Run all ament linters for a package (requires BUILD_TESTING)
colcon build --packages-select antbot_swerve_controller --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select antbot_swerve_controller
colcon test-result --verbose
```

## Testing

There are no unit tests — only ament_lint_auto/ament_lint_common linting tests. The swerve controller's package.xml includes `ament_cmake_gmock` as a test dependency but no gmock tests exist yet.

## Architecture

### Plugin System (ros2_control)

The two core plugins are loaded dynamically via pluginlib:

- **Hardware Interface**: `antbot/hw_interface/BoardInterface` (extends `hardware_interface::SystemInterface`) — declared in `antbot_hw_interface/antbot_hw_interface.xml`
- **Controller**: `antbot_swerve_controller/SwerveDriveController` (extends `controller_interface::ControllerInterface`) — declared in `antbot_swerve_controller/antbot_swerve_controller.xml`

Both implement the ros2_control lifecycle: `on_init()` → `on_configure()` → `on_activate()` → `update()`/`read()`/`write()` → `on_deactivate()`.

### Communication Layer

`antbot_libs` provides shared infrastructure used by `antbot_hw_interface` and `antbot_imu`:
- `Communicator` — Dynamixel Protocol 2.0 wrapper with templated `get_data<T>()` for type-safe register access
- `ControlTableParser` — parses XML control table definitions (via tinyxml2)
- `NodeThread` — node lifecycle management helper

### Hardware Interface Device Abstraction

`antbot_hw_interface` uses a `Device` base class with virtual `read()`/`write()` methods. 8 device classes in `src/device/`: Wheel, Steering, Battery, Cargo, Encoder, Headlight, Ultrasound, Wiper. Each device stores commands/states in `unordered_map<string, double>`.

### Swerve Controller

The largest package (~4,250 LOC across 4 source files). Uses `generate_parameter_library` for YAML-declared parameters (`config/swerve_drive_controller_parameter.yaml`) and `realtime_tools::RealtimePublisher` for real-time safe output. Key files:
- `swerve_drive_controller.cpp` — main controller lifecycle and update loop
- `swerve_motion_control.cpp` — inverse kinematics and motion profiling
- `odometry.cpp` — wheel odometry computation
- `speed_limiter.cpp` — velocity/acceleration/jerk limiting

### Navigation Stack

`antbot_navigation` integrates Nav2 with the swerve platform:
- `slam.launch.py` — online SLAM via slam_toolbox
- `localization.launch.py` — EKF sensor fusion via robot_localization
- `navigation.launch.py` — Nav2 autonomous navigation
- Config files: `nav2_params.yaml`, `ekf.yaml`, `slam_toolbox_params.yaml`

### Package Types

- **ament_cmake** (C++17): `antbot_libs`, `antbot_interfaces`, `antbot_hw_interface`, `antbot_swerve_controller`, `antbot_camera`, `antbot_imu`, `antbot_bringup`, `antbot_description`, `antbot_navigation`
- **ament_python**: `antbot_teleop` (keyboard/joystick teleoperation, swerve sim)

### Custom Interfaces (`antbot_interfaces`)

- **Message**: `CargoStatus.msg` (door/lock status flags)
- **Services**: `CargoCommand.srv`, `DirectRead.srv`, `DirectWrite.srv`, `WiperOperation.srv`

### External Dependencies in Workspace

`DynamixelSDK`, `OrbbecSDK_ROS2`, `coin_d4_driver`, `ublox` — managed via `additional_repos.repos` and cloned by `setting.sh`.

## Key Launch Entry Points

```bash
ros2 launch antbot_bringup bringup.launch.py    # Full robot (all hardware + sensors)
ros2 launch antbot_bringup view.launch.py        # RViz visualization (remote PC)
ros2 launch antbot_description description.launch.py  # URDF preview (no hardware)
ros2 run antbot_teleop teleop_keyboard           # Keyboard teleop
ros2 launch antbot_navigation slam.launch.py     # SLAM mapping
ros2 launch antbot_navigation navigation.launch.py  # Nav2 autonomous navigation
```

## Robot Configuration

- Hardware config XMLs: `antbot_hw_interface/config/` (control table definitions)
- Controller parameters: `antbot_swerve_controller/config/`
- URDF with ros2_control tags: `antbot_description/urdf/ros2_control.xacro`
- Camera config: `antbot_camera/param/camera_config.yaml`
- IMU config: `antbot_imu/config/imu.yaml`
