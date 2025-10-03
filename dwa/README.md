# DWA Navigation Package

Gazebo 시뮬레이션 환경에서 Husky 로봇과 Ouster LiDAR를 활용한 DWA(Dynamic Window Approach) 기반 장애물 회피 네비게이션 패키지입니다.

## 📋 목차

- [개요](#개요)
- [주요 기능](#주요-기능)
- [시스템 구조](#시스템-구조)
- [설치 방법](#설치-방법)
- [실행 방법](#실행-방법)
- [시연 영상](#시연-영상)

## 개요

본 패키지는 Gazebo 시뮬레이션 환경에서 Husky A200 로봇에 Ouster OS1-32 LiDAR를 장착하여 DWA Local Planner를 이용한 자율 주행 네비게이션을 구현합니다.

### 주요 특징

- **Gazebo 시뮬레이션**: 현실적인 물리 엔진 기반 로봇 시뮬레이션
- **Ouster OS1-32 LiDAR**: 3D 포인트 클라우드 기반 장애물 인식
- **DWA Local Planner**: 동적 장애물 회피 경로 계획
- **실시간 Costmap**: Laser scan 기반 장애물 맵 생성

## 주요 기능

### 1. 로봇 플랫폼
- **Husky A200**: Clearpath Robotics의 4륜 차동 구동 로봇
- **Skid Steer Drive**: 실제 하드웨어와 동일한 좌/우 2개 그룹 제어 방식
- **센서**: Ouster OS1-32 LiDAR, IMU, GPS

### 2. 센서 처리 파이프라인
```
Ouster PointCloud2 → VoxelGrid Filter → PointCloud to LaserScan → Move Base
```

### 3. 네비게이션 스택
- **Global Planner**: Global Planner (Dijkstra 기반)
- **Local Planner**: DWA Local Planner (동적 장애물 회피)
- **Costmap**: Rolling window 기반 local/global costmap

## 시스템 구조

### ROS 노드 그래프

<div align="center">
  <img src="docs/images/rqt_graph.png" alt="ROS Node Graph" width="100%"/>
  <p><em>시스템 노드 연결 구조</em></p>
</div>

**주요 노드:**
- `gazebo`: Gazebo 물리 시뮬레이터
- `move_base`: 네비게이션 스택 코어
- `pointcloud_to_laserscan`: 3D → 2D 변환
- `robot_state_publisher`: TF 퍼블리셔
- `voxel_grid`: 포인트 클라우드 다운샘플링

### TF 트리
```
map → odom → base_link → sensors (lidar, imu, wheels)
```

## 설치 방법

### 1. 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-move-base \
    ros-noetic-dwa-local-planner \
    ros-noetic-global-planner \
    ros-noetic-costmap-2d \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-pcl-ros \
    ros-noetic-nodelet \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-tf2-ros
```

### 2. 워크스페이스 설정

```bash
# 워크스페이스 생성
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# 패키지 복사 (이미 존재하는 경우)
# 또는 git clone으로 가져오기

# 빌드
catkin_make

# 환경 설정
source devel/setup.bash
```

## 실행 방법

### Gazebo 시뮬레이션 + DWA 네비게이션

**Terminal 1: Gazebo 환경 및 로봇 스폰**
```bash
roslaunch dwa gazebo_spawn.launch
```

**Terminal 2: DWA 네비게이션 실행**
```bash
roslaunch dwa dwa_test.launch
```

### RViz에서 목표 설정

1. RViz 창에서 상단 메뉴의 **"2D Nav Goal"** 버튼 클릭
2. 목표 지점을 클릭하고 드래그하여 방향 설정
3. 로봇이 자동으로 경로를 계획하고 이동

### 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ouster/points` | `sensor_msgs/PointCloud2` | Ouster LiDAR 원시 데이터 |
| `/scan` | `sensor_msgs/LaserScan` | 2D Laser scan 변환 데이터 |
| `/husky_velocity_controller/cmd_vel` | `geometry_msgs/Twist` | 로봇 속도 명령 |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | 네비게이션 목표 |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 전역 코스트맵 |

## 시연 영상

### RViz 시각화

<div align="center">
  <img src="docs/images/rviz.png" alt="RViz Visualization" width="100%"/>
  <p><em>Ouster LiDAR 포인트 클라우드 및 네비게이션 시각화</em></p>
</div>

**화면 구성:**
- **빨간색/초록색 포인트**: Ouster LiDAR 포인트 클라우드
- **검은색 장애물**: Costmap 장애물 표현
- **노란색 로봇 모델**: Husky A200
- **회색 평면**: Gazebo 환경

### 실행 영상

<div align="center">
  <img src="docs/video/실행.gif" alt="DWA Navigation Demo" width="100%"/>
  <p><em>DWA 네비게이션 실행 데모</em></p>
</div>

## 설정 파일

### Launch 파일

- `gazebo_spawn.launch`: Gazebo 환경 및 로봇 스폰
- `dwa_test.launch`: DWA 네비게이션 + RViz 실행
- `husky_control_nav_localization.launch`: 실제 로봇용 통합 시스템 (Faster-LIO + GPS)

### Config 파일 (`config/`)

- `robot_dynamics_params.yaml`: 로봇 속도/가속도 제한
- `path_planning_params.yaml`: DWA 플래너 파라미터
- `costmap_common_params.yaml`: Costmap 공통 설정
- `local_costmap_params.yaml`: Local costmap 설정
- `global_costmap_params.yaml`: Global costmap 설정

## 기술 상세

### 1. Skid Steer Drive 제어

실제 Husky 하드웨어와 동일하게 4개 바퀴를 좌/우 2개 그룹으로 제어:
- 좌측: `front_left_wheel`, `rear_left_wheel`
- 우측: `front_right_wheel`, `rear_right_wheel`

### 2. Ouster LiDAR 플러그인

Gepetto의 Ouster Gazebo 플러그인 사용:
- GPU Ray Sensor 지원
- 32채널 레이저 스캔
- 0.3~75m 거리 감지

### 3. 포인트 클라우드 처리

```
Ouster (32 beams, 512 samples)
  → VoxelGrid (0.1m)
  → PointCloud2LaserScan
  → DWA Costmap
```

## 트러블슈팅

### 로봇이 움직이지 않는 경우

1. **TF 확인**: `rosrun tf view_frames` 실행 후 TF 트리 확인
2. **토픽 확인**: `rostopic echo /husky_velocity_controller/cmd_vel` 명령어 실행
3. **Costmap 확인**: RViz에서 costmap이 표시되는지 확인

### LiDAR 데이터가 표시되지 않는 경우

1. 빌드 확인: `catkin_make` 재실행
2. 플러그인 경로 확인: `export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/devel/lib`

## 참고 자료

- [Clearpath Husky Documentation](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)
- [Ouster Gazebo Simulation](https://github.com/Gepetto/ouster-gazebo-simulation)

## 라이선스

본 패키지는 연구 및 교육 목적으로 사용됩니다.

## 기여자

- 시뮬레이션 환경 구축 및 DWA 네비게이션 구현
- Ouster LiDAR 플러그인 통합
- Skid Steer Drive 제어 시스템 개발
