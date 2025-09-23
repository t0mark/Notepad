// Faster-LIO 온라인 매핑 실행 파일
// LiDAR와 IMU 데이터를 실시간으로 받아 SLAM을 수행하는 메인 프로그램

#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"

// 궤적 로그 파일 경로 설정 플래그
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

// 프로그램 종료 시그널 핸들러
// Ctrl+C 등의 신호를 받으면 프로그램을 안전하게 종료하도록 플래그 설정
void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    // Google 로깅 시스템 초기화 및 설정
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    // ROS 노드 초기화
    ros::init(argc, argv, "faster_lio");
    ros::NodeHandle nh;

    // LaserMapping 객체 생성 및 ROS 초기화
    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS(nh);

    // 시그널 핸들러 등록 및 실행 주기 설정
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    // 메인 실행 루프 - ROS 메시지를 받아 실시간 매핑 수행
    while (ros::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        ros::spinOnce();
        laser_mapping->Run();
        rate.sleep();
    }

    // 매핑 작업 완료 및 정리
    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    // 성능 통계 출력 및 궤적 저장
    faster_lio::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    return 0;
}
