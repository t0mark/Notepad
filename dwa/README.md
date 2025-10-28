# DWA Navigation Package

## 1. 개요

Husky A200 로봇과 Ouster OS1-32 LiDAR를 활용한 자율 주행 패키지입니다. DWA(Dynamic Window Approach) Local Planner를 기반으로 장애물 회피 네비게이션을 수행합니다.

실제 로봇과 Gazebo 시뮬레이션 환경을 모두 지원하며, 3D PointCloud를 2D LaserScan으로 변환하여 `move_base` 네비게이션 스택에서 활용합니다. Kakao API 등 외부에서 수신한 경로(`nav_msgs/Path`)를 순차적으로 추종하는 `waypoint_manager` 노드가 포함되어 있습니다.

### 주요 기능

- **센서 데이터 처리**
  Ouster 3D PointCloud를 VoxelGrid 필터로 다운샘플링한 후, `pointcloud_to_laserscan`을 통해 2D LaserScan으로 변환하여 네비게이션에 활용합니다.

- **동적 장애물 회피**
  DWA Local Planner를 통해 실시간으로 동적/정적 장애물을 회피하며 경로를 재계획합니다.

- **Costmap 관리**
  Rolling window 방식의 Local/Global Costmap을 활용하여 로봇 주변 환경을 지속적으로 업데이트합니다.

- **지능형 웨이포인트 관리 (`waypoint_manager.py`)**
  - `/kakao/path` 토픽으로 수신된 경로를 웨이포인트로 변환
  - `move_base` Action Client를 통한 웨이포인트 순차 전송
  - Costmap 외부 목표 감지 시 중간 목표 자동 생성
  - 목표 도달 실패 시 재시도 및 건너뛰기 지원
  - RViz를 통한 현재 목표, 남은 경로, 완료된 경로 시각화

## 2. 의존성

본 패키지는 다음 ROS 패키지들에 의존합니다:

**Core ROS**
- `roscpp`, `rospy`

**TF & Transforms**
- `tf`, `tf2_ros`, `tf2_geometry_msgs`

**Message Types**
- `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`

**Navigation Stack**
- `move_base`
- `dwa_local_planner`, `global_planner`
- `costmap_2d`

**Sensor Processing**
- `pcl_ros`, `nodelet`, `pointcloud_to_laserscan`

**Localization & Utils**
- `robot_localization`
- `python3-numpy`

## 3. 설치 방법

### 3.1. ROS 패키지 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-move-base \
    ros-noetic-dwa-local-planner \
    ros-noetic-global-planner \
    ros-noetic-costmap-2d \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-pcl-ros \
    ros-noetic-nodelet \
    ros-noetic-robot-localization \
    ros-noetic-geodesy
```

### 3.2. 워크스페이스 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. 사용법

### 4.1. 시뮬레이션 환경 실행

Gazebo 시뮬레이션과 네비게이션 스택을 함께 실행합니다.

```bash
# world_name: empty, example, city (기본값: example)
roslaunch dwa dwa_simulation.launch [world_name:=<world>]
```

### 4.2. 실제 로봇 실행

실제 로봇에서는 네비게이션 스택만 실행합니다.

```bash
roslaunch dwa dwa_navigation.launch [enable_rviz:=true] [enable_waypoint_manager:=false]
```

**Launch 파라미터**
- `enable_rviz`: RViz 시각화 실행 여부 (기본값: `true`)
- `enable_waypoint_manager`: 웨이포인트 관리 노드 실행 여부 (기본값: `false`, Kakao API 경로 주행 시 `true`로 설정)

### 4.3. RViz에서 목표 설정

1. `dwa_navigation.launch` 또는 `dwa_simulation.launch` 실행
2. RViz 상단의 **"2D Nav Goal"** 버튼 클릭
3. 맵에서 목표 지점 클릭 후 드래그하여 최종 방향 지정
4. 로봇이 자동으로 경로 계획 및 주행 시작

## 5. 패키지 구조

### 5.1. Launch 파일

**`dwa_navigation.launch`**
센서 처리 파이프라인과 `move_base` 노드를 실행하는 메인 네비게이션 런치 파일입니다.

**`dwa_simulation.launch`**
Gazebo 시뮬레이션(`gazebo_spawn_TF_2.launch`)과 네비게이션 스택을 통합 실행합니다.

### 5.2. Configuration 파일

**`costmap_common_params.yaml`**
Global/Local Costmap 공통 설정으로, 로봇 footprint, 장애물 레이어, inflation_radius 등을 정의합니다.

**`global_costmap_params.yaml`**
`map` 프레임 기준 100x100m rolling window를 사용하는 Global Costmap 설정입니다.

**`local_costmap_params.yaml`**
`odom` 프레임 기준 50x50m rolling window를 사용하는 Local Costmap 설정입니다.

**`global_planner_params.yaml`**
Dijkstra 기반 GlobalPlanner 설정으로, `allow_unknown: true`를 통해 미지 영역으로의 경로 계획을 허용합니다.

**`local_planner_params.yaml`**
DWAPlannerROS 설정 파일입니다.
- `max_vel_x: 1.0`, `min_vel_x: -1.0` - 전/후진 속도 (후진 허용)
- `yaw_goal_tolerance: 6.28` - 목표 방향 제약 완전 해제 (360도)
- `path_distance_bias: 64.0` - 경로 추종 가중치 강화

**`move_base_params.yaml`**
move_base 노드 핵심 설정입니다.
- `base_local_planner`: `dwa_local_planner/DWAPlannerROS`
- `base_global_planner`: `global_planner/GlobalPlanner`
- `controller_frequency: 5.0` - 5Hz 제어 주기
- `recovery_behavior_enabled: false` - 복구 동작 비활성화

**`waypoint_manager_params.yaml`**
waypoint_manager 노드 설정입니다.
- `goal_timeout: 60.0` - 단일 웨이포인트 최대 도달 시간(초)
- `max_retries: 3` - 재시도 횟수
- `skip_unreachable: true` - 도달 불가 웨이포인트 건너뛰기

### 5.3. Scripts

**`waypoint_manager.py`**
지능형 웨이포인트 관리 노드입니다.

- **구독**: `/kakao/path` (nav_msgs/Path)
- **발행**: `/kakao/markers` (visualization_msgs/MarkerArray)
- **Action Client**: move_base

**동작 로직**:
1. 새 경로 수신 시 기존 목표 취소 및 웨이포인트 목록 업데이트
2. 로봇 위치에서 가장 가까운 순방향 웨이포인트부터 시작
3. 목표 전송 전 Global Costmap 외부 여부 확인
4. Costmap 외부 목표 감지 시 중간 지점 자동 생성
5. move_base 상태(SUCCEEDED, ABORTED 등)에 따라 재시도/건너뛰기 결정

### 5.4. RViz

**`dwa.rviz`**
로봇 모델, Costmap, 경로, LaserScan, PointCloud 등을 시각화하는 RViz 설정 파일입니다.

## 6. 시스템 아키텍처

### 6.1. 데이터 흐름

```
Ouster LiDAR
    ↓ /ouster/points (PointCloud2)
VoxelGrid Filter
    ↓ /points_filtered (PointCloud2)
PointCloud to LaserScan
    ↓ /scan (LaserScan)
move_base (DWA Planner)
    ↓ /husky_velocity_controller/cmd_vel (Twist)
Robot
```

**단계별 처리**:
1. Ouster LiDAR가 3D 포인트 클라우드를 `/ouster/points` 토픽으로 발행
2. VoxelGrid Filter가 다운샘플링 후 `/points_filtered`로 재발행
3. PointCloud to LaserScan 노드가 2D LaserScan으로 변환 후 `/scan`으로 발행
4. move_base가 `/scan` 데이터로 Costmap 생성 및 DWA 알고리즘 적용
5. 계산된 속도 명령을 `/husky_velocity_controller/cmd_vel`로 전송

### 6.2. TF 트리

```
map → odom → base_link → os_lidar
```