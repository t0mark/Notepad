# Faster-LIO

## Faster-LIO 소개

### 개요
- **개발자**: Chunge Bai, Tao Xiang, Yajie Chen, Haoqian Wang, Fang Zhang, Xiang Gao
- **기반 시스템**: [FastLIO2](https://github.com/hku-mars/FAST_LIO)
- **목적**: 라이다-관성 오도메트리(Lidar-Inertial Odometry)를 위한 경량화된 시스템

### 주요 특징
- **성능 향상**: FastLIO2 대비 약 1.5-2배 속도 향상
- **고성능 처리**:
  - 고체상태 라이다: 1k-2k Hz 달성
  - 일반적인 32라인 회전식 라이다: 100Hz 이상 달성
- **핵심 기술**: 병렬 희소 증분 복셀(Parallel Sparse Incremental Voxels) 사용
- **실시간 처리**: 라이다 포즈 추적과 포인트 클라우드 매핑 동시 수행

### 성능 비교
- **테스트 환경**:
  - AMD R7 5800X 프로세서
  - Intel Xeon Gold 5218 프로세서
- **테스트 데이터**: [NCLT](http://robots.engin.umich.edu/nclt/) 데이터셋
- **지원 라이다**: Avia, Velodyne 32라인, Ouster 32라인 등

## 빌드 방법

### 시스템 요구사항
- **운영체제**: Ubuntu 20.04
- **ROS 버전**: ROS Noetic
- **컴파일러**: C++17 지원 (g++ 9.0 이상)

### 의존성 라이브러리 설치

```bash
# 필수 라이브러리 설치
sudo apt-get install libgoogle-glog-dev \
  libeigen3-dev \
  libpcl-dev \
  libyaml-cpp-dev
```

### 컴파일 과정

#### 1. catkin_make 방식 (권장)
```bash
# catkin 워크스페이스로 이동
cd ~/catkin_ws

# 빌드 실행
catkin_make

# 환경 설정
source devel/setup.bash
```

#### 2. iVox 타입 선택 (선택사항)
```bash
# 기본 linear iVox 사용 (기본값)
catkin_make

# PHC iVox 사용 (고성능 옵션)
catkin_make -DWITH_IVOX_NODE_TYPE_PHC=ON
```

### 빌드 결과
- **라이브러리**: `libfaster_lio.so`
- **실행파일**:
  - `run_mapping_offline`: 오프라인 매핑
  - `run_mapping_online`: 온라인 매핑
- **설정파일**: `config/` 디렉토리의 YAML 파일들

## 실행 방법

### 온라인 모드 (실시간 처리)

#### Ouster 32라인 라이다 사용
```bash
# 터미널 1: Faster-LIO 실행
roslaunch faster_lio mapping_ouster32.launch

# 터미널 2: 데이터 재생
rosbag play your_bag_file.bag
```

#### 다른 라이다 센서 사용
```bash
# Avia 라이다
roslaunch faster_lio mapping_avia.launch

# Velodyne 라이다
roslaunch faster_lio mapping_velodyne.launch
```

#### 실행 과정
1. **roscore 확인**: ROS 마스터가 실행 중인지 확인
2. **launch 파일 실행**: 적절한 센서용 launch 파일 선택
3. **RViz 창 확인**: 실시간 매핑 결과 시각화
4. **rosbag 재생**: 별도 터미널에서 데이터 재생
5. **결과 확인**: RViz에서 궤적과 포인트 클라우드 확인

### 오프라인 모드 (배치 처리)

#### 실행 명령
```bash
# Ouster 32라인 데이터 처리
./build/devel/lib/faster_lio/run_mapping_offline \
  --bag_file /path/to/your_bag_file.bag \
  --config_file ./config/ouster32.yaml

# Avia 데이터 처리
./build/devel/lib/faster_lio/run_mapping_offline \
  --bag_file /path/to/your_avia_bag.bag \
  --config_file ./config/avia.yaml

# Velodyne 데이터 처리
./build/devel/lib/faster_lio/run_mapping_offline \
  --bag_file /path/to/your_velodyne_bag.bag \
  --config_file ./config/velodyne.yaml
```

#### 매개변수 설명
- `--bag_file`: 처리할 rosbag 파일 경로
- `--config_file`: 라이다 센서별 설정 파일 경로

## 출력 정보

### 실시간 성능 모니터링

#### FPS 정보
```shell
I0216 17:16:05.286536 26492 run_mapping_offline.cc:89] Faster LIO average FPS: 1884.6
```
- **의미**: 초당 처리되는 프레임 수
- **목표값**: 
  - 고체상태 라이다: 1000-2000 FPS
  - 회전식 라이다: 100+ FPS

#### 모듈별 처리 시간
```shell
>>> ===== Printing run time =====
> [     IVox Add Points ] average time usage: 0.0147311 ms , called times: 6373
> [     Incremental Mapping ] average time usage: 0.0271787 ms , called times: 6373
> [     ObsModel (IEKF Build Jacobian) ] average time usage: 0.00745852 ms , called times: 25040
> [     ObsModel (Lidar Match) ] average time usage: 0.0298004 ms , called times: 25040
> [ Downsample PointCloud ] average time usage: 0.0224052 ms , called times: 6373
> [ IEKF Solve and Update ] average time usage: 0.342008 ms , called times: 6373
> [ Laser Mapping Single Run ] average time usage: 0.530618 ms , called times: 6387
> [ Preprocess (Livox) ] average time usage: 0.0267813 ms , called times: 6387
> [ Undistort Pcl ] average time usage: 0.0810455 ms , called times: 6375
>>> ===== Printing run time end =====
```

#### 주요 모듈 설명
- **IVox Add Points**: 복셀 맵에 포인트 추가 시간
- **Incremental Mapping**: 증분 매핑 처리 시간
- **ObsModel (IEKF Build Jacobian)**: 야코비안 행렬 구축 시간
- **ObsModel (Lidar Match)**: 라이다 매칭 시간
- **Downsample PointCloud**: 포인트 클라우드 다운샘플링 시간
- **IEKF Solve and Update**: 반복 확장 칼만 필터 계산 시간
- **Laser Mapping Single Run**: 전체 매핑 사이클 시간
- **Preprocess**: 라이다 데이터 전처리 시간
- **Undistort Pcl**: 포인트 클라우드 왜곡 보정 시간

### 저장되는 파일

#### 궤적 파일
- **위치**: `./src/fast_lio2/Log/faster_lio/YYYYMMDD.tum`
- **형식**: TUM 궤적 형식
- **내용**: 타임스탬프, 위치(x,y,z), 방향(qx,qy,qz,qw)
- **예시**:
```
1339439642.462605 0.0 0.0 0.0 0.0 0.0 0.0 1.0
1339439642.512605 0.1 0.0 0.0 0.0 0.0 0.0 1.0
```

#### 포인트 클라우드 파일
- **위치**: `PCD/scans.pcd`
- **형식**: PCL PCD 형식
- **내용**: 전체 누적 포인트 클라우드 맵
- **용도**: 3D 시각화, 추가 분석

### 문제 해결

#### 성능 최적화
- **복셀 크기 조정**: iVox가 불안정한 경우 복셀 크기 증가
- **설정 파일 수정**: `config/` 디렉토리의 YAML 파일에서 파라미터 조정
- **메모리 사용량**: 대용량 데이터셋 처리 시 메모리 모니터링 필요

#### 일반적인 오류
- **컴파일 오류**: `-march=native` 플래그 사용 시 PCL 관련 세그멘테이션 오류 발생 가능
- **의존성 오류**: 필수 라이브러리 누락 시 빌드 실패
- **ROS 환경**: `source devel/setup.bash` 실행 확인 필요