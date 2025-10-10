# DWA Navigation Package

Husky 로봇과 Ouster LiDAR를 활용한 DWA(Dynamic Window Approach) 기반 장애물 회피 네비게이션 패키지입니다.

## 개요

본 패키지는 Husky A200 로봇에 Ouster OS1-32 LiDAR를 장착하여 DWA Local Planner를 이용한 자율 주행 네비게이션을 구현합니다. 실제 로봇과 시뮬레이션 환경 모두에서 사용 가능합니다.

### 주요 특징

- **DWA Local Planner**: 동적 장애물 회피 경로 계획
- **실시간 Costmap**: Laser scan 기반 장애물 맵 생성
- **지능형 목표 관리**: Costmap 범위 밖 목표 자동 처리
- **Navigation Manager**: 장애물 회피를 위한 중간 목표 자동 생성
- **실제 로봇 및 시뮬레이션 지원**: 별도의 gazebo_simulation 패키지와 연동

## 주요 기능

### 1. 센서 처리 파이프라인
```
Ouster PointCloud2 → VoxelGrid Filter → PointCloud to LaserScan → Move Base
```

- **VoxelGrid Filter**: 0.1m leaf size로 포인트 클라우드 다운샘플링
- **PointCloud to LaserScan**: 3D 포인트 클라우드를 2D 레이저 스캔으로 변환
- **높이 필터링**: -0.8m ~ 0.5m 범위만 사용

### 2. 네비게이션 스택
- **Global Planner**: Global Planner (Dijkstra 기반)
- **Local Planner**: DWA Local Planner (동적 장애물 회피)
- **Costmap**: Rolling window 기반 local/global costmap
  - Local: 10x10m, 0.05m 해상도
  - Global: 50x50m, 0.1m 해상도

### 3. Navigation Manager
- **중간 목표 생성**: Costmap 범위 밖 목표를 자동으로 중간 목표로 분할
- **장애물 회피**: 경로 실패 시 좌우로 이동하며 우회 경로 탐색
- **재시도 로직**: 최대 6회 재시도 후 다른 접근 방식 시도

## 설치 방법

### 1. 의존성 설치

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
    ros-noetic-tf2-ros
```

### 2. 워크스페이스 설정

```bash
# 워크스페이스 생성
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# 빌드
catkin_make

# 환경 설정
source devel/setup.bash
```

## 실행 방법

### 시뮬레이션 환경에서 실행

시뮬레이션 환경에서 사용하려면 먼저 `gazebo_simulation` 패키지를 실행해야 합니다:

```bash
# Terminal 1: Gazebo 시뮬레이션 환경 실행
roslaunch gazebo_simulation gazebo_spawn.launch

# Terminal 2: DWA 네비게이션 실행
roslaunch dwa dwa_navigation.launch
```

### 실제 로봇에서 실행

실제 로봇에서는 네비게이션 스택만 실행합니다:

```bash
roslaunch dwa dwa_navigation.launch
```

### RViz에서 목표 설정

1. RViz 창에서 상단 메뉴의 **"2D Nav Goal"** 버튼 클릭
2. 목표 지점을 클릭하고 드래그하여 방향 설정
3. 로봇이 자동으로 경로를 계획하고 이동

## 패키지 구성

```
dwa/
├── launch/
│   └── dwa_navigation.launch  # DWA 네비게이션 실행
├── config/
│   ├── costmap_common_params.yaml
│   ├── local_costmap_params.yaml
│   ├── global_costmap_params.yaml
│   ├── local_planner_params.yaml
│   ├── global_planner_params.yaml
│   └── move_base_params.yaml
├── scripts/
│   └── navigation_manager.py  # 지능형 목표 관리 노드
└── rviz/
    └── dwa.rviz              # RViz 설정 파일
```

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ouster/points` | `sensor_msgs/PointCloud2` | Ouster LiDAR 원시 데이터 |
| `/points_filtered` | `sensor_msgs/PointCloud2` | VoxelGrid 필터링 후 |
| `/scan` | `sensor_msgs/LaserScan` | 2D Laser scan |
| `/husky_velocity_controller/cmd_vel` | `geometry_msgs/Twist` | 로봇 속도 명령 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 네비게이션 목표 |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Local costmap |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Global costmap |

## 설정 파일

### DWA 플래너 파라미터 (local_planner_params.yaml)

```yaml
max_vel_x: 1.0           # 최대 전진 속도 (m/s)
max_vel_theta: 1.5       # 최대 회전 속도 (rad/s)
acc_lim_x: 2.5           # 전진 가속도 제한 (m/s²)
acc_lim_theta: 3.2       # 회전 가속도 제한 (rad/s²)
xy_goal_tolerance: 0.2   # 목표 위치 허용 오차 (m)
yaw_goal_tolerance: 0.17 # 목표 방향 허용 오차 (rad)
```

### Costmap 설정 (costmap_common_params.yaml)

```yaml
obstacle_range: 5.0      # 장애물 감지 범위 (m)
raytrace_range: 6.0      # 레이트레이싱 범위 (m)
inflation_radius: 0.8    # 인플레이션 반경 (m)
footprint: [[-0.5, -0.4], [-0.5, 0.4], [0.5, 0.4], [0.5, -0.4]]
```

## Navigation Manager 기능

### 중간 목표 관리
- 목표가 costmap 범위 밖에 있으면 자동으로 중간 목표 생성
- 로봇-목표 직선과 costmap 경계의 교차점을 중간 목표로 설정
- 중간 목표를 주기적으로 업데이트 (2Hz)

### 장애물 회피
- 경로 계획 실패 감지 ("Failed to get a plan." 메시지 모니터링)
- 2회 연속 실패 시 중간 목표를 좌우로 이동
- 좌우 번갈아 가며 최대 6회 시도
- 실패 시 더 가까운 중간 목표로 재설정

## 관련 패키지

- **gazebo_simulation**: Gazebo 시뮬레이션 환경 제공
- **robot_description**: Husky 로봇 URDF 정의

## 시스템 구조

### 주요 노드
- `move_base`: 네비게이션 스택 코어
- `pointcloud_to_laserscan`: 3D → 2D 변환
- `voxel_grid`: 포인트 클라우드 다운샘플링
- `navigation_manager`: 지능형 목표 관리

### TF 트리
```
map → odom → base_link → os_lidar
```

## 참고 자료

- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)
- [Move Base](http://wiki.ros.org/move_base)
- [Costmap 2D](http://wiki.ros.org/costmap_2d)

## 라이선스

MIT License
