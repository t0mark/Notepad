# robot_description

URDF description package for Husky robot with Ouster LiDAR and GPS sensors.

## 📦 Package Overview

This package provides the robot model definition and handles all static TF transformations for the Husky autonomous robot system.

### TF Responsibility
- **Publishes:** `base_link` → all sensor and wheel frames
- **Uses:** URDF-based `robot_state_publisher`

## 📂 Directory Structure

```
robot_description/
├── urdf/
│   ├── husky.urdf.xacro        # Main robot definition
│   ├── sensors.urdf.xacro      # Ouster LiDAR + GPS sensors
│   └── materials.urdf.xacro    # Visual materials
├── meshes/                     # 3D models (optional)
├── config/
│   └── robot.yaml              # Robot parameters
├── launch/
│   ├── robot_description.launch  # Main launch file
│   └── display.launch           # Visualization with RViz
└── rviz/
    └── robot.rviz              # RViz configuration
```

## 🚀 Usage

### Load Robot Description
```bash
roslaunch robot_description robot_description.launch
```

### Visualize in RViz
```bash
roslaunch robot_description display.launch
```

### Check TF Tree
```bash
rosrun tf view_frames
evince frames.pdf
```

## 🔗 TF Tree Structure

```
base_link
├── os_sensor (Ouster LiDAR mount)
│   ├── os1_lidar (LiDAR data frame)
│   └── os1_imu (IMU frame)
├── gps_link (GPS antenna)
├── front_left_wheel_link
├── front_right_wheel_link
├── rear_left_wheel_link
└── rear_right_wheel_link
```

## 🛠️ Integration with System

### TF Broadcasting Hierarchy
1. **integrated_navigation:** `map` → `odom` (GPS correction)
2. **faster-lio:** `odom` → `base_link` (LiDAR odometry)
3. **robot_description:** `base_link` → sensors/wheels (URDF static)

## 📝 Notes

- All sensor positions are relative to `base_link`
- Ouster LiDAR is mounted 0.3m above base_link
- GPS antenna is mounted at [-0.15, 0, 0.25] relative to base_link
- Wheel joints are continuous for differential drive
