# Kakao API ROS Package

## 1. 개요

웹 브라우저의 카카오맵 UI를 통해 목적지를 설정하고, 카카오내비 API로 생성된 경로를 ROS 토픽으로 발행하는 패키지입니다.

클라이언트(웹)에서 주행에 적합하도록 경로를 필터링하고, 백엔드(ROS)에서 로컬 좌표계로 변환하여 `/kakao/path` 토픽으로 발행합니다.

### 주요 특징

- **웹 기반 UI**
  카카오맵을 통한 직관적인 목적지 설정

- **카카오내비 연동**
  카카오내비 API를 통한 최적 경로 생성

- **경로 최적화**
  웹 UI 슬라이더로 웨이포인트 간격과 곡률을 실시간 조절

- **자동 좌표 변환**
  첫 GPS 수신 위치를 `map` 프레임 원점으로 자동 설정하고, 모든 경로를 로컬 UTM 좌표로 변환

## 2. 데이터 흐름

```
┌──────────────────┐   1. 목적지 클릭   ┌──────────────────┐   2. 경로 요청   ┌────────────────┐
│ 웹 UI (index.html) │────────────────>│ 웹 UI (index.html) │───────────────>│ 카카오내비 API │
└──────────────────┘                  └──────────────────┘                  └────────────────┘
        ^                                      │                                      │
        │ 9. GPS 데이터 전송 (UI 업데이트)         │ 3. 경로 데이터 수신                      │
        │                                      │ (Vertexes)                           │
┌──────────────────┐                  ┌──────────────────┐                  ┌────────────────┐
│ web_server.py    │<──────────────── │ 웹 UI (index.html) │<──────────────── │ 웹 UI (index.html) │
│ (WebSocket 서버) │ 8. /kakao/path 발행└──────────────────┘  4. 경로 최적화    └────────────────┘
└──────────────────┘   (nav_msgs/Path)          │ (필터링)                             ^
        ^                                      │                                      │
        │ 7. UTM 좌표 변환                       │ 5. 최적화된 경로 전송                  │
        │                                      ▼                                      │
┌──────────────────┐                  ┌──────────────────┐                  ┌────────────────┐
│ web_server.py    │<──────────────── │ web_server.py    │<─────────────────┘                  │
│ (Map 원점 기준)  │ 6. WebSocket 수신  │ (WebSocket 서버) │
└──────────────────┘                  └──────────────────┘
```

## 3. 의존성

### 3.1. ROS 패키지

- `rospy`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`

### 3.2. Python 패키지

- `websockets`
- `pyproj`

### 3.3. 설치

```bash
pip install websockets pyproj
```

## 4. API 키 설정

카카오 개발자 사이트에서 발급받은 API 키가 필요합니다.

### 4.1. API 키 발급

[카카오 개발자](https://developers.kakao.com/)에서 애플리케이션을 생성하고 다음 키를 발급받으세요:

- **JavaScript 키** - 웹 SDK용
- **REST API 키** - 내비 API 호출용

### 4.2. 키 설정

`web/index.html` 파일에서 발급받은 키로 수정합니다:

**1. JavaScript SDK 로드**
```html
<script src="https://dapi.kakao.com/v2/maps/sdk.js?appkey=YOUR_JAVASCRIPT_KEY_HERE&libraries=services"></script>
```

**2. REST API 키**
```html
<script>
    fetch(url, {
        method: 'GET',
        headers: {
            'Authorization': 'KakaoAK YOUR_REST_API_KEY_HERE'
        }
    })
</script>
```

## 5. 사용법

### 5.1. 시뮬레이션 모드

GPS 장치 없이 가상 GPS와 RViz를 함께 실행하여 테스트합니다.

```bash
roslaunch kakao_api debug_kakao_api.launch
```

**자동 실행 항목**
- `gps_publisher.py` - 고정 GPS 좌표를 `/ublox/fix`로 발행
- `web_server.py` - WebSocket 서버 실행
- RViz - `/kakao/path` 토픽 시각화
- 웹 브라우저 - `http://localhost:8000` 자동 열림

### 5.2. 실제 주행 모드

실제 GPS 장치(`ublox` 등)에서 `/ublox/fix` 토픽을 발행하는 환경에서 사용합니다.

```bash
roslaunch kakao_api kakao_api.launch
```

**실행 항목**
- `web_server.py`만 실행
- 웹 브라우저에서 `http://localhost:8000`으로 수동 접속 필요

### 5.3. 웹 UI 사용

1. 브라우저에 카카오맵과 GPS 연결 상태 표시
2. 지도에서 목적지 클릭
3. 카카오내비 경로 수신 및 최적화된 웨이포인트 표시
4. `web_server.py`가 경로를 `/kakao/path` 토픽으로 발행
5. RViz에서 `map` 프레임 기준 경로 확인

## 6. 패키지 구조

### 6.1. Launch 파일

**`kakao_api.launch`**
메인 노드(`web_server.py`)만 실행합니다.

**`debug_kakao_api.launch`**
메인 노드, GPS 시뮬레이터, RViz를 함께 실행합니다.

### 6.2. Scripts

**`web_server.py`**
- **HTTP 서버** - `web/index.html` UI 제공 (Port: 8000)
- **WebSocket 서버** - 웹 UI와 GPS/경로 데이터 통신 브릿지 (Port: 8765)
- **ROS 인터페이스** - `/ublox/fix` 구독, 첫 GPS 위치 기준 UTM 좌표계 설정, `/kakao/path` 발행

**`simulator/gps_publisher.py`**
- 테스트용 가상 GPS 데이터를 `/ublox/fix`로 발행
- 실제 GPS 감지 시 자동 발행 중지

### 6.3. Web UI

**`web/index.html`**
- 카카오맵 SDK 기반 프론트엔드 UI
- 목적지 클릭 이벤트 처리 및 카카오내비 API 호출
- 경로 필터링 후 WebSocket으로 `web_server.py`에 전송

### 6.4. RViz

**`rviz/kakao.rviz`**
경로 시각화를 위한 RViz 설정 파일입니다.

## 7. ROS API

### 7.1. 구독 토픽

**`/ublox/fix`** (sensor_msgs/NavSatFix)
- 로봇의 현재 GPS 위치
- 첫 메시지를 `map` 프레임 원점으로 설정
- 웹 UI에 로봇 위치 표시

### 7.2. 발행 토픽

**`/kakao/path`** (nav_msgs/Path)
- 웹 UI로부터 받은 웨이포인트 경로
- `map` 프레임 기준으로 변환되어 발행
- RViz 시각화 또는 내비게이션 노드 입력으로 사용 가능
- Latch 모드로 발행