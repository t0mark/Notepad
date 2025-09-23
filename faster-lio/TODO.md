# Faster-LIO 코드 분석 정리

이 문서는 `faster-lio` ROS 패키지의 코드 구조와 핵심 동작 원리를 분석하고 논의한 내용을 정리합니다.

## 1. 실행 흐름 (Execution Flow)

-   **시작 명령어**: `roslaunch faster_lio mapping_ouster32.launch`
-   **Launch 파일**: `launch/mapping_ouster32.launch`
    -   실행할 노드와 사용할 파라미터 파일을 정의합니다.
-   **설정(Parameter) 파일**: `config/ouster32.yaml`
    -   Lidar 종류, 필터 크기, 외부 파라미터 등 알고리즘의 세부 동작을 제어하는 값들을 정의합니다.
-   **메인 노드 소스코드**: `app/run_mapping_online.cc`
    -   ROS 노드를 초기화하고, 센서 데이터 콜백을 등록하며, SLAM 프로세스를 시작하는 진입점(entry point)입니다.

## 2. Global Map 관련 핵심 로직

Faster-LIO는 두 가지 목적의 Global Map을 메모리 상에서 관리합니다.

### 2.1. 실시간 위치 추정용 Global Map (Working Map)

-   **역할**: 현재 스캔 데이터를 이전에 누적된 지도에 정합(Scan-to-Map)하여 로봇의 현재 위치를 정밀하게 추정하는 데 사용됩니다.
-   **핵심 변수**: `ivox_`
-   **자료구조**: IVox (효율적인 검색을 위한 K-D Tree 기반의 Voxel Grid)
-   **업데이트 위치**:
    -   **파일**: `src/laser_mapping.cc`
    -   **함수**: `MapIncremental()`
    -   **코드**: `544| ivox_->AddPoints(points_to_add);`
-   **중요성**: **필수적**. 이 부분이 없으면 Scan-to-Map 정합이 불가능해져 Localization이 실패하고 드리프트가 급격히 증가합니다.

### 2.2. 파일 저장용 Global Map (Storage Map)

-   **역할**: 최종적으로 pcd 파일로 저장하기 위해 모든 스캔 데이터를 단순히 누적하는 역할.
-   **핵심 변수**: `pcl_wait_save_`
-   **자료구조**: `pcl::PointCloud`
-   **업데이트 위치**:
    -   **파일**: `src/laser_mapping.cc`
    -   **함수**: `PublishFrameWorld()`
    -   **코드**: `742| *pcl_wait_save_ += *laserCloudWorld;`

### 2.3. Global Map 파일 저장

-   **역할**: 메모리에 누적된 `pcl_wait_save_` 맵을 `.pcd` 파일로 저장합니다.
-   **트리거**: ROS 노드가 종료될 때 (e.g., Ctrl+C)
-   **저장 위치**:
    -   **파일**: `src/laser_mapping.cc`
    -   **함수**: `Finish()`
    -   **코드**: `861| pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);`

## 3. 논의된 SLAM 개념

-   **Faster-LIO의 Localization 방식**: **Scan-to-Global-Map** 방식을 사용합니다.
    -   현재 스캔을 이전에 누적된 전체 지도(`ivox_`)에 정합하여 낮은 드리프트(low drift)를 구현합니다.
-   **Loop Closure**: Faster-LIO는 별도의 Loop Closure 기능을 **포함하지 않습니다.** 따라서 Global Map은 대규모 오차 보정용이 아니라, 현재 위치 추정의 기준점(reference)으로 사용됩니다.
-   **Scan-to-Local-Map 방식과의 비교**:
    -   **장점**: 계산량과 메모리 사용이 일정하여 대규모 환경에서 효율적입니다.
    -   **단점**: 기준이 되는 맵이 작고 불안정하여 드리프트가 Scan-to-Global-Map 방식보다 커집니다.
    -   **결론**: Faster-LIO는 `ivox`라는 효율적인 자료구조를 통해 계산량 문제를 해결하고, 낮은 드리프트를 달성하기 위해 Scan-to-Global-Map 방식을 채택했습니다.
