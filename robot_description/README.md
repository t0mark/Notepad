# Robot Description

## 1. 개요

**Clearpath Husky** 로봇에 **Ouster OS1-32 LiDAR**와 **GPS** 센서를 장착한 커스텀 로봇의 URDF 모델을 제공하는 패키지입니다.

로봇의 시각/물리적 속성을 정의하고, Gazebo 시뮬레이션에서 실제 센서와 유사한 데이터를 생성하기 위한 플러그인 설정이 포함되어 있습니다.

## 2. 의존성

### ROS 패키지

**로봇 모델**
- `husky_description` - Husky 로봇 기본 모델
- `ouster_description` - Ouster LiDAR 센서 모델

**시뮬레이션**
- `hector_gazebo_plugins` - Gazebo GPS 플러그인
- `gazebo_ros` - Gazebo ROS 인터페이스

**상태 발행 & 시각화**
- `joint_state_publisher` - Joint states 발행
- `robot_state_publisher` - TF 트리 발행
- `rviz` - 3D 시각화
- `xacro` - URDF 동적 생성

**메시지 타입**
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf`, `urdf`

## 3. 설치 방법

### 3.1. 의존성 패키지 설치

```bash
sudo apt-get update
sudo apt-get install \
    ros-noetic-husky-description \
    ros-noetic-ouster-description \
    ros-noetic-hector-gazebo-plugins
```

### 3.2. 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. 사용법

### 4.1. 로봇 모델 로드 및 TF 발행

로봇 모델을 ROS 파라미터 서버에 로드하고 TF 트리를 발행합니다.

```bash
roslaunch robot_description robot_description.launch
```

**동작**
- `robot_description` 파라미터에 `custom_description.urdf.xacro` 기반 URDF 저장
- `joint_state_publisher`가 비고정 조인트 상태 발행
- `robot_state_publisher`가 모든 링크 간 TF 관계 계산 및 발행

### 4.2. RViz 시각화

Gazebo 없이 로봇의 외형과 TF 트리를 확인합니다.

```bash
roslaunch robot_description display.launch
```

**동작**
- `robot_description.launch` 포함 실행
- RViz 자동 실행 (`robot.rviz` 설정 적용)
- 로봇 모델과 TF 트리 시각화

### 4.3. Gazebo 시뮬레이션 통합

URDF 모델은 Gazebo와 완벽히 호환됩니다. 다른 패키지(예: `gazebo_simulation`)에서 다음과 같이 통합할 수 있습니다.

**1. 로봇 모델 로드**
```xml
<include file="$(find robot_description)/launch/robot_description.launch"/>
```

**2. Gazebo에 로봇 스폰**
```xml
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
      args="-param robot_description -urdf -model husky"/>
```

### 4.4. Gazebo 플러그인

**GPS 센서**
- 플러그인: `libhector_gazebo_ros_gps.so` (`gps.urdf.xacro`)
- 토픽: `/gazebo_time/gps` (sensor_msgs/NavSatFix)

**Ouster LiDAR**
- 플러그인: `OS1-32.urdf.xacro` 설정
- 토픽: `/gazebo_time/points` (sensor_msgs/PointCloud2)
