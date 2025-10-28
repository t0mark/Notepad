# FRNet_ROS

## 1. 개요

FRNet 모델을 활용한 LiDAR 포인트 클라우드 의미론적 분할(Semantic Segmentation) ROS 패키지입니다.

`mmdetection3d` 프레임워크를 기반으로 하며, ROS 환경에서 실시간 포인트 클라우드 처리 및 분할 결과 시각화 기능을 제공합니다.

### 주요 기능

- **실시간 포인트 클라우드 분할**
  ROS 토픽을 통해 수신되는 포인트 클라우드를 실시간으로 처리하고 분할합니다.

- **클래스 필터링**
  특정 클래스 인덱스를 기준으로 포인트를 필터링하여 원하는 객체만 추출하거나 제거할 수 있습니다.

- **처리 주기 제어**
  `process_every_n_scans` 파라미터를 통해 N개 스캔당 한 번씩 처리하여 시스템 부하를 조절합니다.

- **유연한 설정**
  `mmdetection3d` 기반 설정 파일을 통해 다양한 데이터셋과 모델 구성에 대응합니다.

### 동작 방식

`frnet_segmentation_node.py` 노드는 지정된 LiDAR 토픽에서 포인트 클라우드를 구독하고, 사전 훈련된 FRNet 모델로 각 포인트의 클래스(도로, 차량, 보행자 등)를 예측합니다.

## 2. 의존성

### 2.1. ROS 패키지

- `rospy`
- `sensor_msgs`
- `std_msgs`
- `pcl_ros`
- `cv_bridge`

### 2.2. Python 패키지

**Deep Learning**
- `torch`, `torchvision`
- `torch-scatter`

**MMDetection3D 프레임워크**
- `mmengine`
- `mmcv`
- `mmdet`
- `mmdet3d`

**기타**
- `numpy`
- `pyyaml`
- `rospkg`, `catkin_pkg`, `empy`
- `nuscenes-devkit`

### 2.3. 의존성 설치

**PyTorch 설치**
```bash
pip install -U pip setuptools wheel
pip install torch==1.8.1+cu111 torchvision==0.9.1+cu111 -f https://download.pytorch.org/whl/torch_stable.html
pip install torch-scatter==2.0.8 -f https://pytorch-geometric.com/whl/torch-1.8.1+cu111.html
```

**MMDetection3D 프레임워크 설치**
```bash
pip install -U openmim
mim install mmengine==0.9.0
mim install mmcv==2.1.0
mim install mmdet==3.2.0
mim install mmdet3d==1.3.0
```

**CUDA 환경 변수 설정**
```bash
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
```

**ROS 패키지 설치**
```bash
pip install rospkg catkin_pkg empy
sudo apt-get install ros-noetic-cv-bridge ros-noetic-pcl-ros ros-noetic-tf
```

**기타 패키지 설치**
```bash
pip install numpy==1.24.4 pyyaml nuscenes-devkit argparse
```

## 3. 설치 방법

### 3.1. 워크스페이스 설정

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/t0mark/FRNet_ROS
```

### 3.2. 실행 권한 부여

```bash
chmod +x FRNet_ROS/scripts/frnet_segmentation_node.py
```

### 3.3. 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. 사용법

### 4.1. 실행

**세그먼테이션 노드 및 RViz 실행**
```bash
roslaunch frnet_ros frnet.launch
```

**rosbag 테스트**
```bash
rosbag play {rosbag_file}
```

### 4.2. Launch 파라미터

`frnet.launch` 파일에서 다음 인자를 설정할 수 있습니다:

- `lidar_topic` (기본값: `/ouster/points`)
  입력 포인트 클라우드 토픽 이름

- `enable_rviz` (기본값: `true`)
  RViz 실행 여부

- `process_every_n_scans` (기본값: `5`)
  N개 스캔당 한 번씩 모델 추론 실행 (부하 조절)

- `filtered_indices` (기본값: `[1]`)
  분할 결과에서 제거할 클래스 인덱스 리스트
  예: `frnet-school_seg.py`에서 클래스 `1`은 `road`이므로, 기본 설정은 도로 포인트를 제거합니다.

### 4.3. ROS 토픽

**구독 (Subscribe)**
- `/pointcloud_in` (sensor_msgs/PointCloud2)
  `lidar_topic` 파라미터로 리맵핑되는 원본 포인트 클라우드 입력

**발행 (Publish)**
- `/FRNet/points` (sensor_msgs/PointCloud2)
  분할 및 필터링이 완료된 포인트 클라우드 출력

## 5. 설정

### 5.1. 모델 및 가중치

`frnet.launch` 파일에서 다음 ROS 파라미터를 설정합니다:

- `config`
  mmdetection3d 모델 설정 파일(.py) 절대 경로
  예: `$(find frnet_ros)/configs/frnet/frnet-school_seg.py`

- `checkpoint`
  사전 훈련된 모델 가중치 파일(.pth) 절대 경로
  예: `$(find frnet_ros)/checkpoints/checkpoints.pth`

### 5.2. 데이터셋 설정

`configs/_base_/datasets/` 디렉토리에는 다양한 데이터셋 설정이 포함되어 있습니다:

- `mldas_seg.py` - MLDAS 데이터셋
- `nuscenes_seg.py` - NuScenes 데이터셋
- `school_seg.py` - School 데이터셋 (기본 사용)
- `semantickitti_seg.py` - SemanticKITTI 데이터셋

각 파일은 `class_names`와 `labels_map`을 정의하여 데이터셋 라벨을 모델 클래스 인덱스로 매핑합니다.

**school_seg.py 예시**:
```python
class_names = ['unlabeled', 'road']
labels_map = {
    0: 0,
    40: 1
}
```
이 경우 `filtered_indices:=[1]`은 `road` 클래스를 제거합니다.

### 5.3. 모델 아키텍처

- `configs/_base_/models/frnet.py`: FRNet 모델 기본 아키텍처 정의
- `frnet/` 디렉토리: Backbone, Head, Encoder 등 각 모듈의 실제 구현

## 6. 참고사항

### 6.1. Jetson 환경 지원

`scripts/jetson_patch.py` 스크립트는 Jetson 플랫폼에서 발생하는 `torch.distributed` 초기화 오류를 방지하기 위한 패치입니다. `frnet_segmentation_node.py`에서 자동으로 import되어 적용됩니다.