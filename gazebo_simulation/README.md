# Gazebo Simulation Package

Gazebo 시뮬레이션 환경 패키지 - Husky 로봇과 Ouster LiDAR를 위한 시뮬레이션 환경을 제공합니다.

## 개요

본 패키지는 DWA 네비게이션 시스템을 위한 Gazebo 시뮬레이션 환경을 제공합니다. Husky A200 로봇과 Ouster OS1-32 LiDAR가 장착된 시뮬레이션 환경에서 네비게이션 알고리즘을 테스트할 수 있습니다.

## 패키지 구성

```
gazebo_simulation/
├── launch/
│   └── gazebo_spawn.launch       # Gazebo 월드 + 로봇 스폰
├── worlds/
│   ├── empty.world               # 빈 월드
│   ├── example.world             # 예제 환경
│   └── citysim_gazebo.world      # 도시 시뮬레이션 환경
├── models/
│   ├── actor/                    # Gazebo 액터 모델
│   ├── city_terrain/             # 도시 지형 모델
│   └── ocean/                    # 해양 모델
└── scripts/
    └── timestamp_fix.py          # IMU 타임스탬프 변환 스크립트
```

## 주요 기능

### 1. Gazebo 시뮬레이션 환경
- 물리 엔진 기반 로봇 시뮬레이션
- GPU 가속화 지원 (CUDA)
- 커스텀 월드 환경 제공

### 2. 로봇 시뮬레이션
- Husky A200 로봇 모델
- Ouster OS1-32 LiDAR 센서
- IMU 및 GPS 센서

### 3. TF 트리
- `map → odom → base_link` 관계 설정
- 시뮬레이션 환경에서는 map=odom (오차 없음)

## 실행 방법

### 1. 예제 월드 환경 실행 (기본)

```bash
roslaunch gazebo_simulation gazebo_spawn.launch
```

### 2. 빈 월드 환경 실행

```bash
roslaunch gazebo_simulation gazebo_spawn.launch world_name:=empty
```

### 3. 도시 월드 환경 실행

```bash
roslaunch gazebo_simulation gazebo_spawn.launch world_name:=city
```

## Launch 파라미터

### gazebo_spawn.launch

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `world_name` | example | 월드 이름 (empty, example, city) |
| `z_position` | 0.2 | 로봇의 초기 높이 (m) |

**예제:**
```bash
# 빈 월드 + 높이 0.5m
roslaunch gazebo_simulation gazebo_spawn.launch world_name:=empty z_position:=0.5
```

## 환경 변수

- `CUDA_VISIBLE_DEVICES=0`: GPU 가속화 설정
- `GAZEBO_GPU_RAY=1`: LiDAR GPU 레이 트레이싱 활성화
- `GAZEBO_MODEL_PATH`: Gazebo 모델 경로 추가

## 의존성

### ROS 패키지
- `gazebo_ros`
- `gazebo_plugins`
- `gazebo_ros_control`
- `robot_description`
- `tf2_ros`

### 시스템 요구사항
- CUDA 지원 GPU (선택사항, GPU 가속화 사용 시)
- Gazebo 9 이상

## 통합 사용

DWA 네비게이션과 함께 사용하는 경우:

```bash
# Terminal 1: Gazebo 시뮬레이션 실행
roslaunch gazebo_simulation gazebo_spawn.launch

# Terminal 2: DWA 네비게이션 실행
roslaunch dwa dwa_navigation.launch
```

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ouster/points` | `sensor_msgs/PointCloud2` | Ouster LiDAR 데이터 |
| `/imu/data` | `sensor_msgs/Imu` | IMU 데이터 |
| `/husky_velocity_controller/cmd_vel` | `geometry_msgs/Twist` | 로봇 속도 명령 |

## 라이선스

MIT License
