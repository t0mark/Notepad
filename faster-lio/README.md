# Faster-LIO

[![license](https://img.shields.io/badge/License-BSD-brightgreen.svg)](LICENSE)

**Faster-LIO** is a lightweight, tightly-coupled LiDAR-inertial odometry system based on [Fast-LIO2](https://github.com/hku-mars/FAST_LIO). It achieves a 1.5-2x speed improvement over Fast-LIO2 by leveraging parallel sparse incremental voxels (iVox).

This repository provides a ROS implementation of Faster-LIO, designed for real-time performance on computationally limited platforms.

## 1. Features

- **High-Speed Processing**: Utilizes an Iterated Extended Kalman Filter (IEKF) and the iVox data structure for efficient, real-time processing of LiDAR and IMU data.
- **Multi-LiDAR Support**: Compatible with various LiDAR sensors, including Livox, Velodyne, and Ouster.
- **Dual Execution Modes**:
  - **Online Mode**: Performs SLAM in real-time using live sensor data.
  - **Offline Mode**: Processes data from rosbag files for analysis and mapping.
- **Flexible Configuration**: Allows for performance optimization by adjusting parameters in `config` files to suit different sensors and environments.

## 2. Dependencies

### System Requirements
- **OS**: Ubuntu 20.04
- **ROS**: ROS Noetic
- **C++**: C++17 (GCC 9.0 or later)

### Libraries
- **PCL**: 1.10
- **Eigen**: 3.3.7
- **glog**
- **yaml-cpp**

### Dependency Installation
```bash
sudo apt-get update
sudo apt-get install -y libgoogle-glog-dev libeigen3-dev libpcl-dev libyaml-cpp-dev
```

## 3. Installation

### 3.1. Set up Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/gaoxiang12/faster-lio.git
```

### 3.2. Build the Package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3.3. Build Options
You can enable the PHC (Hilbert Curve) iVox type, which may offer better performance in certain environments.

```bash
catkin_make -DWITH_IVOX_NODE_TYPE_PHC=ON
```

## 4. Usage

### 4.1. Online Mode (Real-time)
Run the appropriate launch file for your LiDAR sensor to start real-time SLAM.

**Execution Steps:**

1.  **Start ROS Core**:
    ```bash
    roscore
    ```

2.  **Launch Faster-LIO Node** (in a new terminal):
    - **Livox Avia**:
      ```bash
      roslaunch faster_lio mapping_avia.launch
      ```
    - **Ouster OS1-32**:
      ```bash
      roslaunch faster_lio mapping_ouster32.launch
      ```
    - **Velodyne VLP-16**:
      ```bash
      roslaunch faster_lio mapping_velodyne.launch
      ```

3.  **Play Rosbag** (in a new terminal):
    ```bash
    rosbag play your_lidar_data.bag
    ```

**Visualization**:
By default, RViz is launched automatically (`rviz` argument is `true`) to visualize the map and trajectory.

### 4.2. Offline Mode (Post-processing)
Use the `run_mapping_offline` executable to process rosbag data.

**Command Format**:
```bash
./devel/lib/faster_lio/run_mapping_offline --bag_file <path_to_bag> --config_file <path_to_config>
```

**Example**:
```bash
# Process data from an Ouster 32-line LiDAR
./devel/lib/faster_lio/run_mapping_offline \
  --bag_file /path/to/your_ouster_data.bag \
  --config_file ./config/ouster32.yaml
```

## 5. Configuration Parameters

Key parameters can be adjusted in the `.yaml` files located in the `config/` directory.

### 5.1. Common
- `lid_topic`: Name of the LiDAR data topic.
- `imu_topic`: Name of the IMU data topic.

### 5.2. Preprocess
- `lidar_type`: Type of LiDAR (1: Livox, 2: Velodyne, 3: Ouster).
- `scan_line`: Number of scan lines.
- `blind`: Minimum detection distance (meters).

### 5.3. Mapping
- `extrinsic_T`: Translation vector (x, y, z) between LiDAR and IMU.
- `extrinsic_R`: Rotation matrix (3x3, row-major) between LiDAR and IMU.
- `gyr_cov`, `acc_cov`: Noise covariance for the IMU gyroscope and accelerometer.

### 5.4. PCD Save
- `pcd_save_en`: If `true`, saves the complete point cloud map to a `.pcd` file on exit.
- `interval`: PCD save frequency (in frames). If `-1`, saves only once at the end.

## 6. Outputs

### 6.1. File Outputs
- **Trajectory**:
  - **Location**: `Log/traj.txt`
  - **Format**: TUM format (`timestamp x y z q_x q_y q_z q_w`)
- **Point Cloud Map**:
  - **Location**: `PCD/scans.pcd` (if enabled)
  - **Format**: `.pcd`

### 6.2. ROS Topics
- `/faster_lio/odom`: Odometry information.
- `/faster_lio/path`: Robot's trajectory path.
- `/faster_lio/cloud_registered`: Point cloud registered in the world frame.

## 7. License
This project is licensed under the **BSD License**. See the [LICENSE](LICENSE) file for details.

## 8. Citation
If you use Faster-LIO in your research, please cite the original paper:

```
@article{
bai2022faster,
  title={Faster-LIO: Lightweight Tightly-Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels},
  author={Bai, Chunge and Chen, Zexi and Chen, Yue and Li, Jin and Li, Zhenbo and Liu, Yong},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={11036--11043},
  year={2022},
  publisher={IEEE}
}
```