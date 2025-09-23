// Faster-LIO 레이저 매핑 클래스 구현
// LiDAR와 IMU 센서 데이터를 융합하여 실시간 SLAM을 수행

#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>

#include "laser_mapping.h"
#include "utils.h"

namespace faster_lio {

// ROS 환경에서 LaserMapping 초기화
// 파라미터 로드, ROS 토픽 구독/발행 설정, IVOX 지도 및 칼만 필터 초기화
bool LaserMapping::InitROS(ros::NodeHandle &nh) {
    LoadParams(nh);
    SubAndPubToROS(nh);

    // IVOX 지역 지도 초기화 (파라미터 로드 후 수행)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // Error State Extended Kalman Filter 초기화
    // 23차원 상태벡터용 노이즈 공분산 설정
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    return true;
}

// ROS 없이 YAML 설정 파일로부터 LaserMapping 초기화
// 오프라인 처리나 독립 실행시 사용
bool LaserMapping::InitWithoutROS(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // IVOX 지역 지도 초기화 (파라미터 로드 후 수행)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // Error State Extended Kalman Filter 초기화
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    // 사용 중인 IVOX 타입 로그 출력
    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
    }

    return true;
}

// ROS 파라미터 서버에서 설정값들을 로드
// YAML 설정 파일의 값들을 클래스 멤버 변수들에 할당 (nh로 ROS Master와 연결)
bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    nh.param<bool>("path_save_en", path_save_en_, true);
    nh.param<bool>("publish/path_publish_en", path_pub_en_, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en_, false);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    nh.param<bool>("publish/scan_effect_pub_en", scan_effect_pub_en_, false);
    nh.param<std::string>("publish/tf_imu_frame", tf_imu_frame_, "body");
    nh.param<std::string>("publish/tf_world_frame", tf_world_frame_, "camera_init");

    nh.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh.param<std::string>("map_file_path", map_file_path_, "");
    nh.param<bool>("common/time_sync_en", time_sync_en_, false);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min_, 0.0);
    nh.param<double>("cube_side_length", cube_len_, 200);
    nh.param<float>("mapping/det_range", det_range_, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", preprocess_->Blind(), 0.01);
    nh.param<float>("preprocess/time_scale", preprocess_->TimeScale(), 1e-3);
    nh.param<int>("preprocess/lidar_type", lidar_type, 1);
    nh.param<int>("preprocess/scan_line", preprocess_->NumScans(), 16);
    nh.param<int>("point_filter_num", preprocess_->PointFilterNum(), 2);
    nh.param<bool>("feature_extract_enable", preprocess_->FeatureEnabled(), false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log_, true);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    // PCD 저장 관련 파라미터들
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en_, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval_, -1);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT_, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR_, std::vector<double>());

    // IVOX 복셀 그리드 관련 파라미터들
    nh.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);

    // LiDAR 타입별 전처리 설정
    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    // IVOX 근접 노드 탐색 타입 설정
    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    // 경로 메시지 헤더 초기화
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = tf_world_frame_;

    // 복셀 필터 리프 사이즈 설정
    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    // LiDAR-IMU 외부 보정 행렬 설정
    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    // IMU 처리기에 외부 보정 및 노이즈 공분산 설정
    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

// YAML 설정 파일에서 파라미터들을 직접 로드
// ROS 없이 독립 실행할 때 사용
bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        tf_imu_frame_ = yaml["publish"]["tf_imu_frame"].as<std::string>("body");
        tf_world_frame_ = yaml["publish"]["tf_world_frame"].as<std::string>("camera_init");
        path_save_en_ = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        cube_len_ = yaml["cube_side_length"].as<int>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["point_filter_num"].as<int>();
        preprocess_->FeatureEnabled() = yaml["feature_extract_enable"].as<bool>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    // LiDAR 타입별 전처리 설정
    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    // IVOX 근접 노드 탐색 타입 설정
    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    // 복셀 필터 리프 사이즈 설정
    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    // LiDAR-IMU 외부 보정 행렬 설정
    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    // IMU 처리기에 외부 보정 및 노이즈 공분산 설정
    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    run_in_offline_ = true;
    return true;
}

// ROS 토픽 구독자와 발행자 초기화
// LiDAR 타입에 따라 적절한 메시지 타입으로 구독 설정
void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    std::string lidar_topic, imu_topic;
    nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");

    // LiDAR 타입에 따른 구독자 설정
    if (preprocess_->GetLidarType() == LidarType::AVIA) {
        sub_pcl_ = nh.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr &msg) { LivoxPCLCallBack(msg); });
    } else {
        sub_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr &msg) { StandardPCLCallBack(msg); });
    }

    // IMU 데이터 구독자 설정
    sub_imu_ = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000,
                                              [this](const sensor_msgs::Imu::ConstPtr &msg) { IMUCallBack(msg); });

    // 경로 메시지 헤더 초기화
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = tf_world_frame_;

    // ROS 발행자들 초기화
    pub_laser_cloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    pub_laser_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pub_laser_cloud_effect_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 100000);
    pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 100000);
}

// LaserMapping 생성자
// 포인트클라우드 전처리기와 IMU 처리기 초기화
LaserMapping::LaserMapping() {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

// 메인 실행 함수 - 매 프레임마다 호출되어 SLAM 처리 수행
void LaserMapping::Run() {
    if (!SyncPackages()) {
        return;
    }

    // IMU 데이터 처리, 칼만 필터 예측, 모션 언디스토션 수행
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    // 첫 번째 스캔 처리 - 초기 지도 구축
    if (flg_first_scan_) {
        state_point_ = kf_.get_x();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(&scan_undistort_->points[i], &scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    // 포인트클라우드 다운샘플링 (복셀 필터 적용)
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return;
    }
    // 처리할 데이터 구조들 크기 조정
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP 및 반복 칼만 필터 업데이트
    Timer::Evaluate(
        [&, this]() {
            double solve_H_time = 0;
            // 관측 모델 업데이트 - 최근접점 탐색 및 점-평면 잔차 계산 수행
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // 상태 저장
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // 지역 지도 업데이트
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    // 결과 발행 또는 PCD 파일 저장
    if (run_in_offline_) {
        if (pcd_save_en_) {
            PublishFrameWorld();
        }
        if (path_save_en_) {
            PublishPath(pub_path_);
        }
    } else {
        // 온라인 모드에서 ROS 토픽 발행
        if (pub_odom_aft_mapped_) {
            PublishOdometry(pub_odom_aft_mapped_);
        }
        if (path_pub_en_ || path_save_en_) {
            PublishPath(pub_path_);
        }
        // if (scan_pub_en_ || pcd_save_en_) {
        //     PublishFrameWorld();
        // }
        // if (scan_pub_en_ && scan_body_pub_en_) {
        //     PublishFrameBody(pub_laser_cloud_body_);
        // }
        // if (scan_pub_en_ && scan_effect_pub_en_) {
        //     PublishFrameEffectWorld(pub_laser_cloud_effect_world_);
        // }
    }

    // // 디버그 변수 증가
    // frame_num_++;
}

// 표준 PointCloud2 메시지 콜백 함수
// Ouster, Velodyne 등 표준 ROS PointCloud2 메시지를 처리
void LaserMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            // 시간 역행 체크
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            // 포인트클라우드 전처리 및 버퍼에 저장
            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.push_back(ptr);
            time_buffer_.push_back(msg->header.stamp.toSec());
            last_timestamp_lidar_ = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

// Livox LiDAR 전용 콜백 함수
// Livox CustomMsg 형태의 메시지를 처리하고 시간 동기화 수행
void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            // 시간 역행 체크
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                LOG(WARNING) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            last_timestamp_lidar_ = msg->header.stamp.toSec();

            // IMU-LiDAR 동기화 상태 체크
            if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 && !imu_buffer_.empty() &&
                !lidar_buffer_.empty()) {
                LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
                          << ", lidar header time: " << last_timestamp_lidar_;
            }

            // 자동 시간 동기화 설정
            if (time_sync_en_ && !timediff_set_flg_ && abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 &&
                !imu_buffer_.empty()) {
                timediff_set_flg_ = true;
                timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
                LOG(INFO) << "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu_;
            }

            // 포인트클라우드 전처리 및 버퍼에 저장
            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.emplace_back(ptr);
            time_buffer_.emplace_back(last_timestamp_lidar_);
        },
        "Preprocess (Livox)");

    mtx_buffer_.unlock();
}

// IMU 데이터 콜백 함수
// 수신된 IMU 메시지를 버퍼에 저장하고 시간 동기화 수행
void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count_++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // LiDAR-IMU 시간 동기화 (필요시)
    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    // 시간 역행 체크 (데이터 일관성 보장)
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
}

// LiDAR와 IMU 데이터 동기화 함수
// 시간 순서에 맞게 센서 데이터를 매칭하여 처리 준비
bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    // LiDAR 스캔 데이터 처리
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        // LiDAR 스캔 종료 시간 계산
        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            // 평균 스캔 시간 업데이트
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    // IMU 데이터가 LiDAR 스캔 종료 시간까지 도착했는지 확인
    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    // LiDAR 스캔 시간 동안의 IMU 데이터 수집
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    // 처리된 데이터 버퍼에서 제거
    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

// 현재 상태 정보 출력 (디버그용)
void LaserMapping::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

// 지도 점진적 업데이트 함수
// 새로운 스캔 포인트들을 지도에 선택적으로 추가
void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    // 병렬 처리를 위한 인덱스 배열
    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        // 바디 좌표계에서 월드 좌표계로 변환
        PointBodyToWorld(&(scan_down_body_->points[i]), &(scan_down_world_->points[i]));

        // 지도에 추가할지 결정
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            // 복셀 그리드 중심점 계산
            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            // 다운샘플링이 필요하지 않은 영역 체크
            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            // 지도에 추가 필요성 판단
            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

// LiDAR 포인트클라우드 등록 (매칭) 함수
// ESKF 사용자 정의 관측 모델에서 호출됨
// 점-평면 잔차를 계산하여 칼만 필터 업데이트에 사용
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    // 병렬 처리를 위한 인덱스 벡터 생성
    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            // 현재 상태에서 월드 좌표계로의 변환 행렬 계산
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            // 병렬 처리로 최근접 표면 탐색 및 잔차 계산 수행
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                // 바디 좌표계에서 월드 좌표계로 변환
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    // 지도에서 가장 가까운 표면 포인트들 탐색
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        // 근접 포인트들로부터 평면 방정식 추정
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    // 점-평면 거리 계산
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    // 대응점 유효성 검사
                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

// 경로 발행 함수
// 로봇의 이동 궤적을 ROS Path 메시지로 발행
void LaserMapping::PublishPath(const ros::Publisher pub_path) {
    SetPosestamp(msg_body_pose_);
    msg_body_pose_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    msg_body_pose_.header.frame_id = tf_world_frame_;

    // 경로가 너무 길어지면 RViz가 크래시할 수 있음
    path_.poses.push_back(msg_body_pose_);
    if (run_in_offline_ == false) {
        pub_path.publish(path_);
    }
}

// 오도메트리 발행 함수
// 위치, 자세, 공분산 정보를 포함한 오도메트리 메시지 발행 및 TF 변환 브로드캐스트
void LaserMapping::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped) {
    odom_aft_mapped_.header.frame_id = tf_world_frame_;
    odom_aft_mapped_.child_frame_id = tf_imu_frame_;
    odom_aft_mapped_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    SetPosestamp(odom_aft_mapped_.pose);
    pub_odom_aft_mapped.publish(odom_aft_mapped_);

    // 칼만 필터의 공분산 행렬을 오도메트리 메시지에 설정
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_aft_mapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_aft_mapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_aft_mapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_aft_mapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_aft_mapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_aft_mapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    // TF 변환 브로드캐스트
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_aft_mapped_.pose.pose.position.x, odom_aft_mapped_.pose.pose.position.y,
                                    odom_aft_mapped_.pose.pose.position.z));
    q.setW(odom_aft_mapped_.pose.pose.orientation.w);
    q.setX(odom_aft_mapped_.pose.pose.orientation.x);
    q.setY(odom_aft_mapped_.pose.pose.orientation.y);
    q.setZ(odom_aft_mapped_.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odom_aft_mapped_.header.stamp, tf_world_frame_, tf_imu_frame_));
}

// 월드 좌표계 포인트클라우드 발행 및 PCD 파일 저장
void LaserMapping::PublishFrameWorld() {
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        return;
    }

    PointCloudType::Ptr laserCloudWorld;
    if (dense_pub_en_) {
        // 고밀도 발행 모드: 다운샘플링 전 전체 포인트 사용
        PointCloudType::Ptr laserCloudFullRes(scan_undistort_);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
    } else {
        // 다운샘플링된 포인트 사용
        laserCloudWorld = scan_down_world_;
    }

    // ROS 토픽으로 포인트클라우드 발행
    if (run_in_offline_ == false && scan_pub_en_) {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
        laserCloudmsg.header.frame_id = tf_world_frame_;
        pub_laser_cloud_world_.publish(laserCloudmsg);
        publish_count_ -= options::PUBFRAME_PERIOD;
    }

    // PCD 파일로 지도 저장
    // 주의: 충분한 메모리 확인 필요, 실시간 성능에 영향을 줄 수 있음
    if (pcd_save_en_) {
        *pcl_wait_save_ += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(pcd_index_) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

// 바디 좌표계 포인트클라우드 발행
// LiDAR 좌표계에서 IMU 바디 좌표계로 변환된 포인트들을 발행
void LaserMapping::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
    int size = scan_undistort_->points.size();
    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyLidarToIMU(&scan_undistort_->points[i], &laser_cloud_imu_body->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_imu_body, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = "body";
    pub_laser_cloud_body.publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

// 유효 특징점들의 월드 좌표계 포인트클라우드 발행
// 매칭에 사용된 유효한 특징점들만 시각화
void LaserMapping::PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world) {
    int size = corr_pts_.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyToWorld(corr_pts_[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = tf_world_frame_;
    pub_laser_cloud_effect_world.publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

// 궤적을 파일로 저장
// TUM 형식으로 타임스탬프, 위치, 쿼터니언 저장
void LaserMapping::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto &p : path_.poses) {
        ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

// 현재 상태를 ROS 메시지 형태로 설정
// 위치와 자세 정보를 ROS geometry_msgs에 맞는 형태로 변환
template <typename T>
void LaserMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);
    out.pose.orientation.x = state_point_.rot.coeffs()[0];
    out.pose.orientation.y = state_point_.rot.coeffs()[1];
    out.pose.orientation.z = state_point_.rot.coeffs()[2];
    out.pose.orientation.w = state_point_.rot.coeffs()[3];
}

// 바디 좌표계 포인트를 월드 좌표계로 변환
// LiDAR-IMU 외부 보정 및 현재 위치/자세 적용
void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// 바디 좌표계 벡터를 월드 좌표계로 변환 (오버로드)
// Eigen Vector3f 입력을 받는 버전
void LaserMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

// LiDAR 좌표계 포인트를 IMU 바디 좌표계로 변환
// LiDAR-IMU 외부 보정 변환만 적용
void LaserMapping::PointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

// 매핑 종료 처리 함수
// 남은 PCD 데이터를 최종 저장
void LaserMapping::Finish() {
    // 주의: 충분한 메모리 확인 필요, 실시간 성능에 큰 영향을 줄 수 있음
    if (pcl_wait_save_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        LOG(INFO) << "current scan saved to /PCD/" << file_name;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
    }

    LOG(INFO) << "finish done";
}
}  // namespace faster_lio