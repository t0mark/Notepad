## 설치
```bash
mkdir -p ~/api_ws
cd ~/api_ws

git clone https://github.com/t0mark/Notepad src

source /opt/ros/noetic/setup.bash
catkin_make
echo "source ~/api_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 하드웨어 사용 시
mkdir -p ~/hw_ws
cd ~/hw_ws
git clone https://github.com/t0mark/Notepad -b hardware src
catkin_make
echo "source ~/hw_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

## 순서대로 실행
source ~/api_ws/devel/setup.bash
source ~/hw_ws/devel/setup.bash
```


## husky / husky_robot 의존성 설치
```
# 기본 런타임·제어·네비게이션
sudo apt install -y \
  ros-noetic-roscpp ros-noetic-rospy ros-noetic-roslaunch \
  ros-noetic-controller-manager ros-noetic-diff-drive-controller \
  ros-noetic-joint-state-controller ros-noetic-joint-trajectory-controller \
  ros-noetic-interactive-marker-twist-server ros-noetic-twist-mux \
  ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard \
  ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
  ros-noetic-robot-localization \
  ros-noetic-amcl ros-noetic-gmapping ros-noetic-map-server ros-noetic-move-base \
  ros-noetic-navfn ros-noetic-base-local-planner ros-noetic-dwa-local-planner

# Gazebo 시뮬레이터(옵션)
sudo apt install -y \
  ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins \
  ros-noetic-hector-gazebo-plugins ros-noetic-pointcloud-to-laserscan

# 센서/드라이버(브링업)
sudo apt install -y \
  ros-noetic-imu-filter-madgwick ros-noetic-imu-transformer \
  ros-noetic-lms1xx ros-noetic-microstrain-inertial-driver \
  ros-noetic-nmea-comms ros-noetic-nmea-navsat-driver \
  ros-noetic-realsense2-camera ros-noetic-spinnaker-camera-driver \
  ros-noetic-um6 ros-noetic-um7 ros-noetic-urg-node ros-noetic-velodyne-pointcloud \
  ros-noetic-robot-upstart python3-scipy
```

## ouster_description / ouster_gazebo_plugins / ouster_ros 의존성 설치
```
# ouster_ros (실제 드라이버)  
sudo apt install -y \
  ros-noetic-roscpp ros-noetic-nodelet \
  ros-noetic-sensor-msgs ros-noetic-std-msgs ros-noetic-geometry-msgs \
  ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-message-generation ros-noetic-message-runtime ros-noetic-topic-tools \
  libboost-all-dev libjsoncpp-dev libeigen3-dev \
  libcurl4-openssl-dev libspdlog-dev

# ouster_gazebo_plugins (시뮬레이션용)
sudo apt install -y \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control \
  ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-tf

# ouster_description (URDF/xacro)
sudo apt install -y \
  ros-noetic-urdf ros-noetic-xacro
```

## rtcm_msgs / ublox_f9p 의존성 설치
```
# rtcm_msgs (메시지 생성)
sudo apt install -y \
  ros-noetic-ros-environment \
  ros-noetic-std-msgs \
  ros-noetic-message-generation ros-noetic-message-runtime

# ublox_f9p (ublox, ublox_gps, ublox_msgs, ublox_serialization, ublox_msg_filters)
sudo apt install -y \
  ros-noetic-roscpp ros-noetic-rospy \
  ros-noetic-roscpp-serialization ros-noetic-roslaunch \
  ros-noetic-sensor-msgs ros-noetic-std-msgs \
  ros-noetic-diagnostic-updater \
  ros-noetic-tf \
  ros-noetic-nmea-msgs \
  ros-noetic-ublox-msgs \
  ros-noetic-message-filters
```

## DWA 의존성 설치
```
# DWA 네비게이션/센서 파이프라인
sudo apt install -y \
  ros-noetic-move-base ros-noetic-dwa-local-planner ros-noetic-global-planner \
  ros-noetic-pcl-ros ros-noetic-nodelet ros-noetic-pointcloud-to-laserscan \
  ros-noetic-costmap-2d ros-noetic-dynamic-reconfigure \
  ros-noetic-geographic-msgs ros-noetic-geodesy ros-noetic-robot-localization

# ROS 개발/시각화/TF 툴
sudo apt install -y \
  ros-noetic-rviz \
  ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-std-msgs \
  ros-noetic-move-base-msgs ros-noetic-actionlib-msgs ros-noetic-visualization-msgs

# 기타 의존성
sudo apt install -y python3-numpy python3-tk
```

## Faster-LIO 의존성 설치
```
# Faster-LIO 핵심 ROS 패키지
sudo apt install -y \
  ros-noetic-roscpp ros-noetic-rospy \
  ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-std-msgs \
  ros-noetic-message-generation ros-noetic-message-runtime \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-eigen-conversions \
  ros-noetic-rosbag ros-noetic-rviz

# Faster-LIO 빌드/도구 패키지
sudo apt install -y \
  libgflags-dev libgoogle-glog-dev libtbb-dev \
  libyaml-cpp-dev libeigen3-dev libpcl-dev
```

## gazebo_simulation 의존성 설치
```
# Gazebo 시뮬레이션/로봇 스폰
sudo apt install -y \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-urdf ros-noetic-xacro ros-noetic-pluginlib \
  ros-noetic-roscpp ros-noetic-rospy \
  ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-std-msgs \
  ros-noetic-tf ros-noetic-tf2-ros \
  ros-noetic-robot-state-publisher \
  python3-numpy
```

## ins 의존성 설치
```
sudo apt install -y \
  ros-noetic-roscpp ros-noetic-rospy \
  ros-noetic-robot-localization ros-noetic-hector-trajectory-server \
  ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-geometry-msgs \
  ros-noetic-tf ros-noetic-tf2-ros
```

## robot_description 의존성 설치
```
sudo apt install -y \
  ros-noetic-xacro ros-noetic-urdf \
  ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
  ros-noetic-tf ros-noetic-tf2-ros \
  ros-noetic-rviz
```

## kakao_api 의존성 설치
```
# ROS 메시지/네비/시각화
sudo apt install -y \
  ros-noetic-rospy \
  ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-std-msgs \
  ros-noetic-actionlib-msgs ros-noetic-move-base \
  ros-noetic-rviz

# Python 패키지
pip install -U pip setuptools wheel
pip install pyproj websockets numpy
```

## FRNet_ROS 의존성 설치
```
# ROS 런타임/시각화
sudo apt install -y \
  ros-noetic-rospy \
  ros-noetic-sensor-msgs ros-noetic-std-msgs ros-noetic-pcl-ros \
  ros-noetic-rviz

# PyTorch (+ scatter는 설치한 torch/CUDA와 동일 빌드 선택)
pip install -U pip setuptools wheel
python3 -m pip install --upgrade pip
hash -r
pip install "filelock<3.13" "sympy<1.14" "networkx<3.2" "jinja2<3.2"
pip install torch==2.3.1 torchvision==0.18.1 --index-url https://download.pytorch.org/whl/cu118
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.3.1+cu118.html

# MMDetection3D 스택 (코드에서 mmengine/mmdet3d 사용)
pip install -U openmim --ignore-installed PyYAML
mim install mmcv==2.2.0
mim install mmdet mmdet3d

# 기타 Python 패키지
pip install numpy pyyaml rospkg catkin_pkg empy argparse
```