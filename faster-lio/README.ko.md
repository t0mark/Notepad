# FasterLIO

이것은 Chunge Bai, Tao Xiang, Yajie Chen, Haoqian Wang, Fang Zhang, Xiang Gao가 개발한 FasterLIO의 코드 저장소입니다.

Faster-LIO는 라이다 포즈 추적과 포인트 클라우드 매핑을 위한 경량화된 라이다-관성 오도메트리입니다. [FastLIO2](https://github.com/hku-mars/FAST_LIO)를 기반으로 개발되었으며 약 1.5-2배의 속도 향상을 제공합니다. 고체상태 라이다에서는 거의 1k-2k Hz를, 일반적인 32라인 회전식 라이다에서는 100Hz 이상을 달성합니다. 자세한 내용은 [우리의 논문](./doc/faster-lio.pdf)을 참조하세요.

아래는 FastLIO2와 [NCLT](http://robots.engin.umich.edu/nclt/)의 avia 및 velodyne 32 회전식 라이다로 테스트한 FPS입니다. "AMD"는 AMD R7 5800X로, "Intel"은 Intel Xeon Gold 5218로 테스트되었습니다.

<div  align="center">  
<img src="https://github.com/gaoxiang12/faster-lio/blob/main/doc/fps-fasterlio.png" width = "1080" align=center />
</div>

<div  align="center">  
<img src="https://github.com/gaoxiang12/faster-lio/blob/main/doc/faster-lio-nclt.png" width = "1080" align=center />
</div>

# 빠른 시작

## Docker 
```bash
git clone 
cd faster-lio/docker
docker-compose build
docker-compose up
```  
`docker-compose up` 명령을 입력하기 전에, 로컬 터미널에서 `xhost +local:docker`를 입력하여 도커가 호스트의 Xserver와 통신할 수 있도록 설정해야 합니다.

`.env` 파일에서 `bag` 파일 경로를 업데이트한 후 실행하세요.
```bash
catkin_make
source devel/setup.bash
roslaunch faster_lio mapping_avia.launch
```
다른 터미널을 열고 `bag` 파일을 재생하세요.

### 문제점
`QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'` 

**걱정하지 마세요, 정상적으로 실행됩니다**. `source /etc/profile&& source /opt/ros/noetic/setup.bash && source devel/setup.bash`를 실행한 후 `roslaunch xxx`를 실행할 수 있습니다.

## 의존성

FasterLIO는 Ubuntu 18.04와 Ubuntu 20.04에서 테스트되었습니다. 컴파일 전에 다음 라이브러리를 설치하세요.

1. ROS (melodic 또는 noetic)
2. glog: ```sudo apt-get install libgoogle-glog-dev```
3. eigen: ```sudo apt-get install libeigen3-dev```
4. pcl: ```sudo apt-get install libpcl-dev```
5. yaml-cpp: ```sudo apt-get install libyaml-cpp-dev```

## 컴파일

FasterLIO는 plain cmake 또는 catkin_make로 컴파일할 수 있습니다. Ubuntu 20.04에서는 컴파일 과정이 비교적 간단합니다.

1. Plain cmake 

다음 명령을 사용하여 FasterLIO를 빌드하세요:

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

참고: iVox 타입은 컴파일 시점에 cmake로 지정되어야 합니다. 기본적으로 linear iVox를 사용합니다.
PHC iVox로 FasterLIO를 빌드하려면 ```cmake .. -DWITH_IVOX_NODE_TYPE_PHC=ON```을 사용하세요.

2. catkin_make

이 저장소를 catkin 워크스페이스(예: ~/catkin_ws/src)에 클론한 후, 위의 cmake 명령 대신 catkin_make를 사용하세요. catkin_make 매개변수에서도 iVox 타입을 지정할 수 있습니다.

컴파일 후에는 libfaster_lio.so와 두 개의 실행 파일을 얻게 됩니다. plain cmake 빌드를 선택했다면 ./build/devel/lib/faster_lio에 위치하고, catkin_make를 사용했다면 rosrun과 roslaunch로 실행할 수 있습니다.

3. Ubuntu 18.04 이하에서 컴파일

FasterLIO는 cpp 17을 표준으로 사용하므로(더 높은 버전의 g++가 필요함), 컴파일러를 업그레이드하고 이 저장소의 thirdparty에 제공된 이전 tbb 라이브러리를 사용해야 합니다. 다음 단계를 따르세요:

- g++ 컴파일러를 9.0 이상으로 업그레이드:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9
cd /usr/bin
sudo rm gcc g++
sudo ln -s gcc-9 gcc
sudo ln -s g++-9 g++
```

- thirdparty에서 tbb 라이브러리 압축 해제:

```bash
cd thirdparty
tar -xvf tbb2018_20170726oss_lin.tgz
```

- cmake 단계에서 tbb 디렉토리 지정:

```bash
cd ../build
cmake .. -DCUSTOM_TBB_DIR=`pwd`/../thirdparty/tbb2018_20170726oss
```

그 다음 make로 FasterLIO를 빌드하세요.

## 데이터셋 준비

컴퓨터에 avia/nclt bags를 다운로드하세요:

- [avia bags](https://drive.google.com/drive/folders/1YL5MQVYgAM8oAWUm7e3OGXZBPKkanmY1?usp=sharing)
- [nclt bags](https://drive.google.com/drive/folders/1VBK5idI1oyW0GC_I_Hxh63aqam3nocNK)

데이터셋 링크가 오래되었습니다. SAD 데이터셋의 NCLT bags를 사용하세요: 
BaiduYun: https://pan.baidu.com/s/1ELOcF1UTKdfiKBAaXnE8sQ?pwd=feky 접근 코드: feky
OneDrive: https://1drv.ms/u/s!AgNFVSzSYXMahcEZejoUwCaHRcactQ?e=YsOYy2

## FasterLIO 실행

FasterLIO는 일반적인 바이너리 프로그램처럼 호출할 수 있습니다. 다른 ros 프로그램처럼 오프라인 모드 또는 온라인 모드로 실행할 수 있습니다.

- 오프라인 모드

매개변수와 함께 run_mapping_offline을 호출하여 bag 파일과 config 파일을 지정하세요:

```bash
./build/devel/lib/faster_lio/run_mapping_offline --bag_file your_avia_bag_file --config_file ./config/avia.yaml
```

avia의 경우입니다. NCLT의 경우 다음과 같이 실행하세요:

```bash
./build/devel/lib/faster_lio/run_mapping_offline --bag_file your_nclt_bag_file --config_file ./config/velodyne.yaml
```

"your avia bag file"을 컴퓨터의 경로로 바꿔주세요. FasterLIO는 종료 시 FPS와 시간 사용량을 출력합니다:

```shell
I0216 17:16:05.286536 26492 run_mapping_offline.cc:89] Faster LIO average FPS: 1884.6
I0216 17:16:05.286549 26492 run_mapping_offline.cc:91] save trajectory to: ./src/fast_lio2/Log/faster_lio/20120615.tum
I0216 17:16:05.286706 26492 utils.h:52] >>> ===== Printing run time =====
I0216 17:16:05.286711 26492 utils.h:54] > [     IVox Add Points ] average time usage: 0.0147311 ms , called times: 6373
I0216 17:16:05.286721 26492 utils.h:54] > [     Incremental Mapping ] average time usage: 0.0271787 ms , called times: 6373
I0216 17:16:05.286731 26492 utils.h:54] > [     ObsModel (IEKF Build Jacobian) ] average time usage: 0.00745852 ms , called times: 25040
I0216 17:16:05.286752 26492 utils.h:54] > [     ObsModel (Lidar Match) ] average time usage: 0.0298004 ms , called times: 25040
I0216 17:16:05.286775 26492 utils.h:54] > [ Downsample PointCloud ] average time usage: 0.0224052 ms , called times: 6373
I0216 17:16:05.286784 26492 utils.h:54] > [ IEKF Solve and Update ] average time usage: 0.342008 ms , called times: 6373
I0216 17:16:05.286792 26492 utils.h:54] > [ Laser Mapping Single Run ] average time usage: 0.530618 ms , called times: 6387
I0216 17:16:05.286800 26492 utils.h:54] > [ Preprocess (Livox) ] average time usage: 0.0267813 ms , called times: 6387
I0216 17:16:05.286808 26492 utils.h:54] > [ Undistort Pcl ] average time usage: 0.0810455 ms , called times: 6375
I0216 17:16:05.286816 26492 utils.h:59] >>> ===== Printing run time end =====
```

포인트 클라우드는 기본적으로 PCD/scans.pcd에 저장됩니다.

- 온라인 모드 
 
온라인 모드는 rosrun/roslaunch/직접 호출을 통해 실행할 수 있습니다. roslaunch를 예시로 사용합니다:

1. faster-lio 실행: ```roslaunch faster_lio mapping_avia.launch``` 이것은 rviz 창을 제공합니다.
2. ```rosbag play your bag file```을 사용하여 bags를 재생하고 온라인 출력을 확인하세요.

# 감사의 말

- [FastLIO2](https://github.com/hku-mars/FAST_LIO), LOAM의 저자들에게 훌륭한 작업에 대해 감사드립니다.
- 학술 작업에서 FasterLIO를 사용하고 있다면 우리의 작업을 인용해 주세요. Bibtex가 여기에 제공됩니다 (조기 액세스 버전):

```
@ARTICLE{9718203,  
author={Bai, Chunge and Xiao, Tao and Chen, Yajie and Wang, Haoqian and Zhang, Fang and Gao, Xiang},  
journal={IEEE Robotics and Automation Letters},   
title={Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels},   
year={2022},  
volume={7},  
number={2},  
pages={4861-4868},  
doi={10.1109/LRA.2022.3152830}}
```

- 이 작업은 Idriver+ Technologies Co. Ltd.에서 지원합니다.

# 알려진 문제점

- iVox는 복셀 크기에 다소 민감합니다. 응용 프로그램에서 FasterLIO가 불안정하다면 복셀 크기를 늘려주세요.
- cmake에서 -march=native 설정은 PCL 포인트 클라우드 해제 단계에서 코어 덤프를 일으킬 수 있습니다 (Ubuntu 20.04, gcc 11, AMD 5800X)(Ubuntu 18.04, gcc 9, i7-10750H). 참조: 
  https://stackoverflow.com/questions/61278204/segmentation-fault-when-deallocating-pclpointcloudpclpointxyzptr