# INS (Integrated Navigation System)

## 1. 개요

`ins` 패키지는 `robot_localization`을 활용하여 다중 센서(Lidar Odometry, Wheel Odometry, IMU, GPS) 데이터를 융합하는 통합 항법 시스템입니다.

### 주요 시나리오

**1. Lidar Odometry + GPS/IMU 융합** (`ins_faster_lio.launch`)
- `faster-lio` 등 Lidar SLAM 결과를 GPS/IMU로 보정
- 전역 위치 정확도 향상 및 드리프트 보정

**2. Wheel Odometry + GPS/IMU 융합** (`ins_wheel.launch`)
- Husky 등 바퀴 로봇의 Odometry를 IMU/GPS와 융합
- Dual EKF 구조로 로컬/전역 좌표계 모두 제공

## 2. 의존성

### 필수 ROS 패키지

- `robot_localization` - EKF 센서 융합 필터
- `hector_trajectory_server` - 경로 시각화

### 센서 드라이버

- `ouster-ros` - IMU 데이터
- `ublox` - GPS 데이터
- `husky_robot` - Wheel Odometry (Wheel 융합 모드)

### 선택 사항

- `faster-lio` - Lidar SLAM (Lidar 융합 모드 사용 시)

## 3. 설치 방법

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. 사용법

### 4.1. Lidar Odometry + GPS/IMU 융합

`faster-lio` Odometry를 GPS/IMU와 융합하여 `map` → `odom` 변환을 생성하고 드리프트를 보정합니다.

**선행 조건**
- `faster-lio` 노드 실행 중
- GPS 드라이버: `/ublox/fix` 발행
- IMU 드라이버: `/ouster/imu` 발행

**실행**
```bash
roslaunch ins ins_faster_lio.launch
```

**입력 토픽**
- `/faster_lio/odom` (nav_msgs/Odometry) - Lidar Odometry
- `/ouster/imu` (sensor_msgs/Imu) - IMU 데이터
- `/ublox/fix` (sensor_msgs/NavSatFix) - GPS 데이터

**출력**
- `map` → `odom` TF - EKF 계산 전역 보정 변환
- `/global/odom` (nav_msgs/Odometry) - 융합 Odometry
- `/global/trajectory` (nav_msgs/Path) - RViz 시각화 경로

### 4.2. Wheel Odometry + GPS/IMU 융합

Dual EKF 구조를 사용하여 바퀴 로봇의 로컬/전역 위치를 모두 추정합니다.

**EKF 구조**

1. **Local EKF** (`odom` 프레임)
   - Wheel Odometry + IMU 융합
   - 연속적이고 드리프트가 적은 `odom` → `base_link` 변환 생성

2. **Global EKF** (`map` 프레임)
   - Local EKF 결과 + GPS 융합
   - 전역 좌표계에서 위치 보정, `map` → `odom` 변환 생성

**선행 조건**
- 로봇 컨트롤러: `/husky_velocity_controller/odom` 발행
- GPS 드라이버: `/ublox/fix` 발행
- IMU 드라이버: `/ouster/imu` 발행

**실행**
```bash
roslaunch ins ins_wheel.launch
```

**입력 토픽**
- `/husky_velocity_controller/odom` (nav_msgs/Odometry) - Wheel Odometry
- `/ouster/imu` (sensor_msgs/Imu) - IMU 데이터
- `/ublox/fix` (sensor_msgs/NavSatFix) - GPS 데이터

**출력**
- `odom` → `base_link` TF - Local EKF 로컬 변환
- `map` → `odom` TF - Global EKF 전역 보정 변환
- `/global/odom` (nav_msgs/Odometry) - 융합 Odometry
- `/global/trajectory` (nav_msgs/Path) - RViz 시각화 경로
