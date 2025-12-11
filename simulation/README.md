# Gazebo Simulation

## 1. 개요

Gazebo 시뮬레이션 환경을 위한 ROS 패키지입니다. Husky 로봇과 Ouster LiDAR를 포함한 시뮬레이션 환경을 제공하며, `dwa_navigation` 패키지와 함께 사용하여 네비게이션 알고리즘을 테스트할 수 있습니다.

## 2. 의존성

### ROS 패키지
- `roscpp`
- `rospy`
- `tf`
- `tf2_ros`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `gazebo_ros`
- `gazebo_plugins`
- `gazebo_ros_control`
- `hardware_interface`
- `pluginlib`
- `urdf`
- `xacro`
- `robot_description`

### Python 3 패키지
- `numpy`

### 시스템 요구사항
- `Gazebo 9` 이상
- `CUDA` 지원 GPU (GPU 가속화 사용 시)

## 3. 설치 방법

```bash
catkin_make
```

## 4. 사용법

### 4.1 Gazebo 시뮬레이션 실행

Gazebo 시뮬레이션 월드와 Husky 로봇을 스폰합니다.

- **기본 월드 (custom.world) 실행**
  ```bash
  roslaunch simulation gazebo_spawn.launch
  ```

- **다른 월드 실행**
  `world_name` 파라미터를 사용하여 원하는 월드를 지정할 수 있습니다.
  - `empty`
  - `example`
  - `citysim`

  ```bash
  # citysim 월드 실행
  roslaunch simulation gazebo_spawn.launch world_name:=citysim
  ```

### 4.2 TF Publisher 실행

`gazebo_spawn_TF.launch` 또는 `gazebo_spawn_TF_2.launch` 파일을 사용하여 `map`과 `odom` 사이의 TF 관계를 설정할 수 있습니다.

- **`gazebo_spawn_TF.launch`**
  - `map` -> `odom` `static_transform_publisher`를 실행합니다.
  - `robot_description`에서 `odom` -> `base_link` TF를 발행하지 않습니다.

  ```bash
  roslaunch simulation gazebo_spawn_TF.launch
  ```

- **`gazebo_spawn_TF_2.launch`**
  - `map` -> `odom` `static_transform_publisher`를 실행합니다.
  - `robot_description`에서 `odom` -> `base_link` TF를 발행합니다.

  ```bash
  roslaunch simulation gazebo_spawn_TF_2.launch
  ```

### 4.3 `timestamp_fix.py`

Gazebo에서 발행하는 센서 데이터의 타임스탬프를 현재 ROS 시간으로 변환하여 다시 발행하는 노드입니다.

- **실행**
  `gazebo_spawn.launch` 파일에 포함되어 있어 별도로 실행할 필요가 없습니다.

- **구독 토픽**
  - `/gazebo_time/imu` (`sensor_msgs/Imu`)
  - `/gazebo_time/gps` (`sensor_msgs/NavSatFix`)
  - `/gazebo_time/points` (`sensor_msgs/PointCloud2`)

- **발행 토픽**
  - `/ouster/imu` (`sensor_msgs/Imu`)
  - `/ublox/fix` (`sensor_msgs/NavSatFix`)
  - `/ouster/points` (`sensor_msgs/PointCloud2`)