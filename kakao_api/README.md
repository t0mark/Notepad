# kakao_api ROS 패키지

## 📌 개요

웹 브라우저의 카카오맵에서 목적지를 클릭하면, 카카오 내비게이션 API로 경로를 받아와 ROS 토픽으로 발행하는 패키지입니다.

**핵심 특징**:
- ✅ **좌표 변환 없음** - GPS 좌표(위경도)를 그대로 발행
- ✅ **프레임 독립적** - map/odom 프레임 정의 불필요
- ✅ **경량 설계** - 웹 ↔ ROS 브릿지 역할만 수행

다른 노드(예: INS, navsat_transform)가 GPS → map 좌표 변환을 담당합니다.

---

## 🏗️ 패키지 구조

```
kakao_api/
├── scripts/
│   ├── web_server.py         # HTTP + WebSocket 서버 (웹 ↔ ROS 브릿지)
│   ├── route_planner.py      # GPS 웨이포인트 변환 (JSON → GeoPath)
│   └── gps_debug.py          # 테스트용 GPS 발행기
├── web/
│   └── index.html            # 카카오맵 웹 UI
└── launch/
    └── debug.launch          # 디버깅용 런치 파일
```

---

## 📦 파일 역할

### `scripts/web_server.py`
- **HTTP 서버** (8000): `index.html` 제공
- **GPS WebSocket** (8765): ROS `/ublox/fix` → 웹으로 스트리밍
- **Waypoint WebSocket** (8766): 웹 → ROS `/kakao/goal` 발행

### `scripts/route_planner.py`
- `/kakao/goal` (JSON) → `/kakao/waypoints` (GeoPath) 변환
- GPS 좌표 검증 및 ROS 메시지 타입 변환
- **좌표 변환 안 함** (위경도 그대로 유지)

### `scripts/gps_debug.py`
- 테스트용 가상 GPS 발행 (`/ublox/fix`)
- Map 프레임 원점 정의 (INS 없을 때)
- 외부 GPS 감지 시 자동 중지

### `web/index.html`
- 카카오맵 표시 + 로봇 위치 마커
- 목적지 클릭 → 카카오 내비 API 호출
- 경로 최적화 (거리/곡률 기반 필터링)
- WebSocket으로 서버와 통신

---

## 🔄 데이터 흐름

```
┌────────────────────────────────────────────────────┐
│ 웹 브라우저 (카카오맵)                              │
│  - 로봇 위치 표시                                   │
│  - 목적지 설정 → 카카오 API 호출                    │
└────────────────┬───────────────────────────────────┘
                 │ WebSocket
                 ↓
┌────────────────────────────────────────────────────┐
│ web_server.py                                      │
│  - ROS ↔ 웹 브릿지                                 │
└────────────────┬───────────────────────────────────┘
                 │ /kakao/goal (String, JSON)
                 ↓
┌────────────────────────────────────────────────────┐
│ route_planner.py                                   │
│  - JSON → GeoPath 변환                             │
│  - GPS 좌표 검증                                    │
└────────────────┬───────────────────────────────────┘
                 │ /kakao/waypoints (GeoPath)
                 ↓
┌────────────────────────────────────────────────────┐
│ 다른 노드 (navsat_transform, waypoint_follower 등) │
│  - GPS → map 좌표 변환                             │
│  - 경로 추종                                        │
└────────────────────────────────────────────────────┘
```

---

## 📡 ROS 토픽

### 입력
- `/ublox/fix` (sensor_msgs/NavSatFix) - 로봇 현재 GPS

### 출력
- `/kakao/goal` (std_msgs/String) - 웹에서 받은 목적지 (JSON)
- `/kakao/waypoints` (geographic_msgs/GeoPath) - GPS 웨이포인트 배열
- `/kakao/status` (std_msgs/String) - 노드 상태 정보

---

## 🚀 설치 및 실행

### 의존성 설치

```bash
# ROS 패키지
sudo apt-get install ros-noetic-geographic-msgs

# Python 패키지
pip3 install websockets
```

### 실행 방법

#### 1️⃣ GPS 장비 있을 때
```bash
# web_server + route_planner 실행
rosrun kakao_api web_server.py
rosrun kakao_api route_planner.py
```

자동으로 브라우저가 열리고 `http://localhost:8000/index.html` 접속됩니다.

#### 2️⃣ GPS 없이 테스트 (디버깅)
```bash
roslaunch kakao_api debug.launch
```

가상 GPS 발행기 포함하여 실행됩니다.

---

## 🔧 카카오 API 설정

`web/index.html`에서 카카오 API 키 설정 필요:

```javascript
// index.html 내부
const KAKAO_API_KEY = 'YOUR_API_KEY_HERE';
const KAKAO_REST_API_KEY = 'YOUR_REST_API_KEY_HERE';
```

API 키 발급: https://developers.kakao.com/

---

## 💡 사용 예시

### 웹에서 목적지 설정
1. 브라우저에서 지도 확인
2. 목적지 클릭
3. 카카오 API가 경로 생성
4. 슬라이더로 웨이포인트 간격 조절
5. "경로 전송" 버튼 클릭

### ROS에서 웨이포인트 확인
```bash
# 웨이포인트 토픽 확인
rostopic echo /kakao/waypoints

# 상태 확인
rostopic echo /kakao/status
```

---

## 🏗️ 시스템 통합 예시

INS(robot_localization)와 함께 사용:

```
/kakao/waypoints (GeoPath)
    ↓
navsat_transform (GPS → map 좌표 변환)
    ↓
waypoint_follower (경로 추종)
    ↓
move_base (내비게이션)
```

---

## 📝 참고사항

- **좌표 변환**: 이 패키지는 GPS 좌표를 변환하지 않습니다. navsat_transform 또는 별도 노드 사용 필요.
- **Map 프레임**: `gps_debug.py`는 독립 실행용이며, INS 사용 시 필요 없음.
- **Latch 모드**: `/kakao/waypoints`는 늦게 시작한 노드도 마지막 경로를 받을 수 있도록 설정됨.

---

## 🐛 문제 해결

### 웹 페이지가 안 열려요
```bash
# 수동으로 접속
firefox http://localhost:8000/index.html
```

### GPS 데이터가 안 와요
```bash
# GPS 토픽 확인
rostopic hz /ublox/fix

# 디버그 모드로 테스트
rosrun kakao_api gps_debug.py
```

### WebSocket 연결 실패
```bash
# 포트 확인
netstat -tuln | grep -E '8765|8766|8000'

# 노드 재시작
rosnode kill /web_server
rosrun kakao_api web_server.py
```
