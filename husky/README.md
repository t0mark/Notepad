husky
=====

Clearpath Husky를 위한 공통 ROS 패키지 모음으로, 시뮬레이션과 실제 로봇 운용 모두에서 사용할 수 있습니다.

 - husky_control : 제어 설정
 - husky_description : 로봇 설명(URDF)
 - husky_msgs : 메시 정의
 - husky_navigation : 내비게이션 설정 및 데모

Husky 사용법과 튜토리얼은 [Robots/Husky](http://wiki.ros.org/Robots/Husky)를 참고하세요.

맞춤형 Husky 설명 또는 시뮬레이션을 만들고 싶다면 [husky_customization](https://github.com/husky/husky_customization)을 포크하세요.

husky_desktop
=============

Clearpath Husky용 데스크톱 ROS 패키지로, 그래픽 관련 의존성이 포함될 수 있습니다.

 - husky_viz : 시각화(RViz) 구성 및 브링업

Husky 사용법과 튜토리얼은 http://wiki.ros.org/Robots/Husky 를 참고하세요.

husky_simulator
==============

Clearpath Husky를 위한 시뮬레이터 ROS 패키지입니다.

 - husky_gazebo : Gazebo 플러그인 정의와 로봇 URDF 확장

Husky 사용법과 튜토리얼은 http://wiki.ros.org/Robots/Husky 를 참고하세요.

폴더 구조
--------

```
husky/
|-- LICENSE
|-- README.md
|-- husky_control/
|   |-- CHANGELOG.rst
|   |-- CMakeLists.txt
|   |-- config/
|   |   |-- control.yaml
|   |   |-- empty.yaml
|   |   |-- gazebo_interface.yaml
|   |   |-- localization.yaml
|   |   |-- public_interface.yaml
|   |   |-- teleop_logitech.yaml
|   |   |-- teleop_ps4.yaml
|   |   `-- twist_mux.yaml
|   |-- launch/
|   |   |-- control.launch
|   |   `-- teleop.launch
|   `-- package.xml
|-- husky_description/
|   |-- CHANGELOG.rst
|   |-- CMakeLists.txt
|   |-- README.md
|   |-- launch/
|   |   `-- description.launch
|   |-- meshes/
|   |   |-- accessories/
|   |   |   |-- 300_mm_sensor_arch.dae
|   |   |   |-- 300_mm_sensor_arch.stl
|   |   |   |-- 510_mm_sensor_arch.dae
|   |   |   |-- 510_mm_sensor_arch.stl
|   |   |   |-- hokuyo_ust10.stl
|   |   |   |-- lidar_mount.stl
|   |   |   |-- lms1xx_mount.dae
|   |   |   |-- lms1xx_mount.stl
|   |   |   `-- wibotic_bumper.stl
|   |   |-- base_link.dae
|   |   |-- base_link.stl
|   |   |-- bracket_extension_20.stl
|   |   |-- bracket_extension_40.stl
|   |   |-- bracket_extension_60.stl
|   |   |-- bracket_extension_80.stl
|   |   |-- bracket_horizontal.stl
|   |   |-- bracket_horizontal_large.stl
|   |   |-- bracket_vertical.stl
|   |   |-- bumper.dae
|   |   |-- bumper_extension.dae
|   |   |-- large_top_plate.dae
|   |   |-- large_top_plate_collision.stl
|   |   |-- pacs_full_riser.stl
|   |   |-- pacs_partial_riser.stl
|   |   |-- pacs_top_plate.stl
|   |   |-- top_chassis.dae
|   |   |-- top_chassis.stl
|   |   |-- top_plate.dae
|   |   |-- top_plate.stl
|   |   |-- user_rail.dae
|   |   |-- user_rail.stl
|   |   |-- wheel.dae
|   |   `-- wheel.stl
|   |-- package.xml
|   `-- urdf/
|       |-- accessories/
|       |   |-- flir_blackfly_mount.urdf.xacro
|       |   |-- hokuyo_ust10.urdf.xacro
|       |   |-- intel_realsense.urdf.xacro
|       |   |-- sensor_arch.urdf.xacro
|       |   |-- sick_lms1xx_mount.urdf.xacro
|       |   `-- vlp16_mount.urdf.xacro
|       |-- decorations.urdf.xacro
|       |-- empty.urdf
|       |-- husky.urdf.xacro
|       |-- pacs.urdf.xacro
|       `-- wheel.urdf.xacro
|-- husky_desktop/
|   |-- CHANGELOG.rst
|   |-- CMakeLists.txt
|   `-- package.xml
|-- husky_gazebo/
|   |-- CHANGELOG.rst
|   |-- CMakeLists.txt
|   |-- launch/
|   |   |-- empty_world.launch
|   |   |-- husky_playpen.launch
|   |   |-- multi_husky_playpen.launch
|   |   |-- playpen.launch
|   |   |-- realsense.launch
|   |   `-- spawn_husky.launch
|   |-- package.xml
|   `-- worlds/
|       `-- clearpath_playpen.world
|-- husky_msgs/
|   |-- CHANGELOG.rst
|   |-- CMakeLists.txt
|   |-- msg/
|   |   `-- HuskyStatus.msg
|   `-- package.xml
|-- husky_navigation/
|   |-- CHANGELOG.rst
|   |-- CMakeLists.txt
|   |-- config/
|   |   |-- costmap_common.yaml
|   |   |-- costmap_exploration.yaml
|   |   |-- costmap_global_laser.yaml
|   |   |-- costmap_global_static.yaml
|   |   |-- costmap_local.yaml
|   |   `-- planner.yaml
|   |-- launch/
|   |   |-- amcl.launch
|   |   |-- amcl_demo.launch
|   |   |-- exploration.launch
|   |   |-- exploration_demo.launch
|   |   |-- gmapping.launch
|   |   |-- gmapping_demo.launch
|   |   |-- move_base.launch
|   |   `-- move_base_mapless_demo.launch
|   |-- maps/
|   |   |-- playpen_map.pgm
|   |   `-- playpen_map.yaml
|   `-- package.xml
|-- husky_simulator/
|   |-- CHANGELOG.rst
|   |-- CMakeLists.txt
|   `-- package.xml
`-- husky_viz/
    |-- CHANGELOG.rst
    |-- CMakeLists.txt
    |-- launch/
    |   |-- view_diagnostics.launch
    |   |-- view_model.launch
    |   `-- view_robot.launch
    |-- package.xml
    |-- rqt/
    |   `-- diagnostics.perspective
    `-- rviz/
        |-- model.rviz
        `-- robot.rviz
```

파일 설명
--------

- **공통 파일**
  - `LICENSE`: Clearpath Husky 패키지 라이선스 전문.
  - `README.md`: 상단 개요와 본 폴더 구조/설명 문서.

- **husky_control/**
  - `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml`: 패키지 이력과 빌드·의존성 정의.
  - `config/`: 기본 제어(`control.yaml`), 텔레옵 프로파일, Gazebo/Localization 설정 등을 묶은 YAML 모음.
  - `launch/`: 제어 스택과 텔레옵 노드를 기동하는 런치 파일.

- **husky_description/**
  - 코어 파일: `CHANGELOG.rst`, `CMakeLists.txt`, `README.md`, `package.xml`, `launch/description.launch`.
  - `meshes/`: 베이스 링크, 바퀴, 상판과 다양한 브래킷·센서 마운트를 DAE/STL로 제공.
    - `meshes/accessories/`: LiDAR, 카메라, 센서 아치 등 추가 장비용 메시.
  - `urdf/`: Husky 기본/확장 URDF(Xacro)와 PACS, 장식, 휠 서브모듈 정의.
    - `urdf/accessories/`: 센서 마운트 Xacro 매크로 집합.

- **husky_desktop/**
  - 그래픽 의존성이 있는 데스크톱 환경 구성용 메타 패키지.
  - `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml`로 구성.

- **husky_gazebo/**
  - Gazebo 통합 패키지: `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml` 포함.
  - `launch/`: 빈 월드, Playpen, 다중 로봇, RealSense 등 다양한 시뮬레이션 시나리오 런치.
  - `worlds/clearpath_playpen.world`: 기본 제공 Gazebo 월드.

- **husky_msgs/**
  - `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml`: 메시 패키지 메타데이터.
  - `msg/HuskyStatus.msg`: 배터리, 온도 등 상태 정보를 담는 단일 메시 정의.

- **husky_navigation/**
  - 핵심 파일: `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml`.
  - `config/`: 비용지도, 플래너, 탐사 설정 YAML 묶음.
  - `launch/`: AMCL, Gmapping, 탐사, move_base 등 내비게이션 파이프라인 런치.
  - `maps/`: Playpen 환경용 지도(`playpen_map.pgm/.yaml`).

- **husky_simulator/**
  - 시뮬레이터용 메타 패키지. `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml`로 구성.

- **husky_viz/**
  - 시각화·진단 패키지: `CHANGELOG.rst`, `CMakeLists.txt`, `package.xml`.
  - `launch/`: 모델 뷰, 로봇 뷰, 진단용 런치 파일.
  - `rqt/diagnostics.perspective`: rqt 진단 워크스페이스 설정.
  - `rviz/`: 모델/로봇 관찰을 위한 RViz 설정 파일.
