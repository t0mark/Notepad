# FRNet_ROS
## 설명


## 종속성 파일 설치
``` bash
# Pytorch 패키지
pip install -U pip setuptools wheel
pip install torch==1.8.1+cu111 torchvision==0.9.1+cu111 -f https://download.pytorch.org/whl/torch_stable.html

pip install torch-scatter==2.0.8 -f https://pytorch-geometric.com/whl/torch-1.8.1+cu111.html
pip install nuscenes-devkit
pip install argparse pyyaml

# MIM 패키지 설치
pip3 install -U openmim

mim install mmengine==0.9.0
mim install mmcv==2.1.0
mim install mmdet==3.2.0
mim install mmdet3d==1.3.0

export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# ROS 패키지 설치
pip install rospkg catkin_pkg empy

sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-pcl-ros
sudo apt-get install ros-noetic-tf

# 기타 패키지
pip install numpy==1.24.4 pyyaml
```

## 빌드
``` bash
# ROS 워크스페이스 생성 (이미 있는 경우 생략)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# FRNet 저장소 복제
git clone https://github.com/t0mark/FRNet_ROS

chmod +x FRNet_ROS/scripts/frnet_segmentation_node.py

cd ..
catkin_make
source devel/setup.bash
```

## 사용
``` bash
# 터미널 1
roscore

# 터미널 2
roslaunch frnet_ros frnet_seg.launch

# 터미널 3
rosbag play {rosbag file} 
```
