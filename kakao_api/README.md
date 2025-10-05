# 개요
KaKao Navi를 사용한 

# 주요 적용 사항
## main branch 참조 [[Link](https://github.com/kyeonghyeon0314/gazebo_dwa)]
- husky에 HDL-32E 추가
- husky_velodyne.launch 수행내용
- husky_dwa_navigation
- NavigationManager

#### 기술적 부분
faster-lio에서 map을 생성하는 부분을 없애고 x-y평면으로 정상여하여 pose를 추출하게 변형한다. [[팀원2](https://github.com/Cascio99)분이 제작]


# TODO
```
mkdir -p ~/husky_ws/src
cd husky_ws
catkin_make

cd src
git clone https://github.com/kyeonghyeon0314/gazebo_dwa.git -b gps_localization

# 필수 의존성 설치
sudo apt-get install ros-noetic-gazebo-ros \
                     ros-noetic-roscpp \
                     ros-noetic-rospy \
                     ros-noetic-sensor-msgs \
                     ros-noetic-std-msgs \
                     ros-noetic-geometry-msgs \
                     ros-noetic-nav-msgs \
                     ros-noetic-tf \
                     ros-noetic-tf2-ros \
                     ros-noetic-tf2-geometry-msgs \
                     ros-noetic-velodyne-description \
                     ros-noetic-velodyne-gazebo-plugins \
                     ros-noetic-dwa-local-planner \
                     ros-noetic-global-planner \
                     ros-noetic-move-base \
                     ros-noetic-pcl-ros \
                     ros-noetic-nodelet \
                     ros-noetic-pointcloud-to-laserscan \
                     ros-noetic-costmap-2d \
                     ros-noetic-dynamic-reconfigure \
                     ros-noetic-urdf \
                     ros-noetic-xacro
pip install utm

cd ..
catkin_make

# 테스트
roslaunch kakao_api debug.launch

# 실행
roslaunch kakao_api kakao_api.launch
```