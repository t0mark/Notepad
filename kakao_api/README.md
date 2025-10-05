# kakao_api ROS 패키지

## 1. 개요

`kakao_api`는 ROS(Robot Operating System) 환경에서 카카오맵 API와 내비게이션 API를 활용하여 로봇의 자율 주행 경로를 생성하고 관리하는 패키지입니다.

사용자는 웹 브라우저에 표시된 카카오맵에서 목적지를 클릭하기만 하면, 카카오내비 API를 통해 최적의 경로를 받아오고, 이 경로를 로봇이 주행할 수 있는 웨이포인트(Waypoints) 형태로 변환하여 ROS 내비게이션 스택으로 전달합니다.

## 2. 주요 기능

- **웹 기반 인터페이스**: 카카오맵을 통해 직관적으로 목적지를 설정하고 로봇의 현재 위치를 실시간으로 확인할 수 있습니다.
- **카카오내비 연동**: 카카오내비 API를 호출하여 자동차 최단 거리 경로를 생성합니다.
- **경로 최적화**: API로부터 받은 상세 경로를 로봇 주행에 적합하도록 간격과 곡률을 고려하여 지능적으로 필터링하고 최적화합니다.
- **좌표 변환**: 웹에서 받은 GPS 좌표(위도, 경도)를 로봇의 로컬 `map` 좌표계로 자동 변환합니다.
- **ROS 통합**: 변환된 경로는 표준 `geometry_msgs/PoseArray` 메시지 형태로 `/global_waypoints` 토픽에 발행되어 `move_base`와 같은 내비게이션 스택과 쉽게 연동됩니다.

## 3. 아키텍처

패키지는 다음과 같은 노드와 웹 인터페이스의 상호작용으로 동작합니다.

```
+-------------------------+      (1) GPS 좌표 전송      +---------------------+
|                         | <----------------------- |                     |
|   Web UI (index.html)   |      (WebSocket)         |   web_server.py     |
| (KaKao Map, WebSocket)  |                          | (HTTP, WebSocket)   |
|                         | -----------------------> |                     |
+-------------------------+   (2) Waypoints(경로) 전송  +----------+----------+
           ^                                                     | (3) /waypoints_gps 토픽 발행
           | (6) 실시간 GPS 데이터 수신                             |
           |                                                     v
+----------+----------+      (5) /ublox/fix 토픽 구독      +--------------------------+
|                     | <----------------------- |                          |
|  GPS 수신기 (하드웨어)  |                          | kakao_route_planner.py   |
| (또는 debug_gps_pub) | -----------------------> | (좌표 변환 및 경로 발행) |
+---------------------+   (4) 첫 GPS를 원점으로 설정      +--------------------------+
                                                                   |
                                                                   | (7) /global_waypoints 토픽 발행
                                                                   v
                                                        +------------------------+
                                                        |                        |
                                                        | ROS Navigation Stack   |
                                                        | (e.g., move_base)      |
                                                        +------------------------+
```

1.  **`web_server.py`**는 로봇의 현재 GPS(`NavSatFix`) 데이터를 `/ublox/fix` 토픽에서 구독하여 웹 UI로 실시간 전송합니다.
2.  사용자가 웹 UI에서 목적지를 클릭하면, 카카오내비 API로 경로를 요청하고, 최적화된 웨이포인트를 `web_server.py`로 전송합니다.
3.  **`web_server.py`**는 수신한 웨이포인트를 `/waypoints_gps` 토픽(JSON 문자열)으로 발행합니다.
4.  **`kakao_route_planner.py`**는 처음 수신된 GPS 좌표를 `map` 좌표계의 원점(0, 0)으로 설정합니다.
5.  **`kakao_route_planner.py`**는 `/waypoints_gps` 토픽을 구독하고, GPS 웨이포인트를 `map` 좌표계로 변환합니다.
6.  변환된 경로는 `PoseArray` 메시지 형태로 `/global_waypoints` 토픽에 발행됩니다.
7.  `move_base`와 같은 ROS 내비게이션 스택은 이 웨이포인트를 따라 자율 주행을 수행합니다.

## 4. 노드 상세 설명

### `scripts/web_server.py`

- **HTTP 서버**: `8000`번 포트에서 웹 UI(`web/index.html`)를 제공합니다.
- **WebSocket 서버**:
  - **GPS 전송 (포트 `8765`)**: ROS의 `/ublox/fix` 토픽을 구독하여 현재 로봇의 GPS 위치를 웹 UI로 실시간 스트리밍합니다.
  - **경로 수신 (포트 `8766`)**: 웹 UI에서 생성된 경로(웨이포인트)를 수신하여 `/waypoints_gps` ROS 토픽으로 발행합니다.
- 노드 실행 시 자동으로 기본 웹 브라우저를 열어 UI에 접속합니다.

### `scripts/kakao_route_planner.py`

- **좌표계 원점 설정**: 시스템 시작 후 처음 수신하는 GPS(`ublox/fix`) 데이터를 `map` 좌표계의 원점으로 설정합니다.
- **좌표 변환**: `/waypoints_gps` 토픽으로 들어온 경로(GPS)를 UTM 좌표계로 변환한 뒤, 설정된 원점을 기준으로 `map` 좌표계로 변환합니다.
- **경로 발행**: 최종 변환된 경로는 `geometry_msgs/PoseArray` 타입으로 `/global_waypoints` 토픽에 발행하여 내비게이션 스택이 사용할 수 있도록 합니다.

### `scripts/debug_gps_publisher.py`

- 디버깅 및 테스트를 위한 노드입니다.
- 실제 GPS 장비가 없을 때, 고정된 GPS 좌표를 `/ublox/fix` 토픽으로 1Hz 주기로 발행하여 시스템을 테스트할 수 있게 합니다.

### `web/index.html`

- 카카오맵 API를 사용하여 지도를 표시하는 단일 페이지 웹 애플리케이션입니다.
- WebSocket을 통해 `web_server.py`와 통신하여 로봇의 현재 위치를 지도에 마커로 표시합니다.
- 사용자가 지도를 클릭하여 목적지를 설정하면, 카카오내비 길찾기 API를 호출하여 경로 데이터를 요청합니다.
- **자체 경로 최적화 알고리즘**:
  - API로부터 받은 원본 경로(수백~수천 개의 촘촘한 점)를 그대로 사용하지 않고, 로봇 주행에 맞게 필터링합니다.
  - **거리 기반 필터링**: 직선 구간에서는 웨이포인트 간격을 넓게 설정합니다.
  - **곡률 기반 필터링**: 급격한 커브 구간에서는 웨이포인트를 촘촘하게 유지하여 부드러운 주행이 가능하도록 합니다.
  - UI의 슬라이더를 통해 **간격**과 **곡선 정밀도**를 실시간으로 조절할 수 있습니다.
- 최적화된 최종 웨이포인트를 WebSocket을 통해 `web_server.py`로 전송합니다.

## 5. 실행 방법

### 사전 준비

```bash
# 의존성 패키지 설치
sudo apt-get install ros-noetic-move-base
pip install utm websockets
```

### 1. 실제 주행 (`kakao_api.launch`)

실제 GPS 장비와 로봇을 구동할 때 사용합니다.

```bash
roslaunch kakao_api kakao_api.launch
```

- `web_server.py`와 `kakao_route_planner.py` 노드를 실행합니다.
- `use_sim_time` 파라미터가 `true`로 설정되어 시뮬레이션 시간과 동기화됩니다.

### 2. 디버깅 및 테스트 (`debug.launch`)

실제 GPS 없이 기능을 테스트할 때 사용합니다.

```bash
roslaunch kakao_api debug.launch
```

- `kakao_api.launch`의 기능에 더해 `debug_gps_publisher.py` 노드를 추가로 실행합니다.
- 이 노드가 가상의 GPS 데이터를 발행하므로, GPS 장비 없이도 시스템의 동작을 확인할 수 있습니다.
- `use_sim_time` 파라미터가 `false`로 설정됩니다.

## 6. 주요 토픽 및 파라미터

### 발행 토픽

- `/global_waypoints` (`geometry_msgs/PoseArray`): 내비게이션 스택이 사용할 최종 경로.
- `/waypoints_gps` (`std_msgs/String`): 웹 UI에서 받은 원본 GPS 경로 (JSON).
- `/kakao_route/status` (`std_msgs/String`): `kakao_route_planner` 노드의 상태 정보.

### 구독 토픽

- `/ublox/fix` (`sensor_msgs/NavSatFix`): 로봇의 현재 GPS 위치.
- `/waypoints_gps` (`std_msgs/String`): `kakao_route_planner`가 구독하는 경로 토픽.

### 주요 파라미터

- `use_sim_time` (`bool`): ROS 시간을 시뮬레이션 시간 기준으로 사용할지 여부.
