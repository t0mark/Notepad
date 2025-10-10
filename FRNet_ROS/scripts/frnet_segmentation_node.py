#!/usr/bin/env python3
import os
import sys
import numpy as np
import torch
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import mmengine
import struct
from mmdet3d.structures import Det3DDataSample, PointData
from mmdet3d.apis import init_model
import traceback

# FRNet 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class FRNetSegmentation:
    def __init__(self):
        rospy.init_node('frnet_segmentation_node', anonymous=True)

        # 파라미터 로드
        self.config_path = rospy.get_param('~config', '/home/tomark/catkin_ws/src/frnet_ros/configs/frnet/frnet-semantickitti_seg.py')
        self.checkpoint_path = rospy.get_param('~checkpoint', '/home/tomark/catkin_ws/src/frnet_ros/checkpoints/frnet_semantickitti.pth')
        self.device = rospy.get_param('~device', 'cuda:0')

        # 디버깅 모드 및 설정
        self.debug = rospy.get_param('~debug', True)

        # 필터링할 클래스 인덱스 리스트 (SemanticKITTI 클래스 기준)
        self.filtered_indices = rospy.get_param('~filtered_indices', [0])

        # 모델 초기화
        rospy.loginfo("모델 초기화 중...")
        try:
            self.model = init_model(self.config_path, self.checkpoint_path, device=self.device)
            rospy.loginfo("모델 초기화 완료")
        except Exception as e:
            rospy.logerr(f"모델 초기화 실패: {e}")
            rospy.logerr(traceback.format_exc())
            sys.exit(1)

        # 구독자 및 발행자 설정
        self.sub = rospy.Subscriber('pointcloud_in', PointCloud2, self.pointcloud_callback, queue_size=1)
        self.pub = rospy.Publisher('/FRNet/points', PointCloud2, queue_size=1)

        rospy.loginfo(f"FRNet 세그먼테이션 노드가 준비되었습니다. 필터링 인덱스: {self.filtered_indices}")

    def convert_to_semantickitti_format(self, points):
        """Ouster LiDAR 포인트 클라우드를 SemanticKITTI 형식으로 변환"""
        # SemanticKITTI 데이터는 보통 (x, y, z, intensity) 형식
        # 불필요한 필드 제거하고 포맷 맞추기
        if points.shape[1] > 4:
            points = points[:, :4]  # (x, y, z, intensity)만 사용

        # intensity 값 정규화 (SemanticKITTI 형식과 유사하게)
        max_intensity = np.max(points[:, 3])
        if max_intensity > 0:
            points[:, 3] = points[:, 3] / max_intensity

        return points

    def pointcloud_callback(self, msg):
        rospy.loginfo("포인트 클라우드 데이터 수신")

        try:
            # PointCloud2 메시지에서 직접 포인트 데이터 추출
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                points.append([p[0], p[1], p[2], p[3]])

            if not points:
                rospy.logwarn("포인트 클라우드가 비어 있습니다.")
                return

            points = np.array(points, dtype=np.float32)

            # 디버깅 정보 출력
            if self.debug:
                rospy.loginfo(f"포인트 클라우드 형태: {points.shape}, 타입: {points.dtype}")

            # 이상치 제거 (너무 멀리 있는 포인트 제거)
            distance = np.sqrt(np.sum(points[:, :3] ** 2, axis=1))
            valid_indices = distance < 100.0  # 100m 범위 내 포인트만 사용
            points = points[valid_indices]

            # SemanticKITTI 형식으로 변환
            points = self.convert_to_semantickitti_format(points)

            # 포인트 클라우드 추론
            pred_labels = self.infer_pointcloud(points)

            # 필터링된 포인트 클라우드 생성 (필터링 인덱스에 해당하지 않는 포인트만 유지)
            filtered_mask = np.isin(pred_labels, self.filtered_indices, invert=True)
            filtered_points = points[filtered_mask]

            if self.debug:
                rospy.loginfo(f"원본 포인트 수: {points.shape[0]}, 필터링 후: {filtered_points.shape[0]}")

            # 결과를 ROS 메시지로 변환하여 발행
            self.publish_filtered_pointcloud(msg.header, filtered_points)

        except Exception as e:
            rospy.logerr(f"포인트 클라우드 처리 중 오류 발생: {e}")
            rospy.logerr(traceback.format_exc())

    def infer_pointcloud(self, points):
        try:
            with torch.no_grad():
                # 포인트 클라우드를 텐서로 변환
                points_tensor = torch.from_numpy(points).float().to(self.device)

                # Det3DDataSample 생성 및 필요한 속성 설정
                data_sample = Det3DDataSample(metainfo=dict(num_points=points.shape[0]))

                # PointData 객체로 gt_pts_seg 설정 (빈 마스크)
                data_sample.gt_pts_seg = PointData()
                data_sample.gt_pts_seg.pts_semantic_mask = torch.zeros(points.shape[0], dtype=torch.long).to(self.device)

                # 모델 입력 형식으로 데이터 구성
                model_inputs = dict(
                    inputs=dict(points=[points_tensor]),
                    data_samples=[data_sample]
                )

                # 데이터 전처리 수행
                try:
                    processed_data = self.model.data_preprocessor(model_inputs, False)
                    if self.debug:
                        rospy.loginfo("데이터 전처리 완료")
                except Exception as e:
                    rospy.logerr(f"데이터 전처리 중 오류: {e}")
                    return np.zeros(points.shape[0], dtype=np.int32)

                # 모델 추론
                try:
                    results = self.model.forward(**processed_data, mode='predict')
                    if self.debug:
                        rospy.loginfo("모델 추론 완료")
                except Exception as e:
                    rospy.logerr(f"모델 추론 중 오류: {e}")
                    return np.zeros(points.shape[0], dtype=np.int32)

                # 예측된 레이블
                pred_labels = results[0].pred_pts_seg.pts_semantic_mask.cpu().numpy()

                # 디버깅 정보
                if self.debug:
                    unique_labels, counts = np.unique(pred_labels, return_counts=True)
                    rospy.loginfo(f"세그먼테이션 결과 - 레이블 분포: {dict(zip(unique_labels, counts))}")

                return pred_labels
        except Exception as e:
            rospy.logerr(f"추론 과정 중 예외 발생: {e}")
            rospy.logerr(traceback.format_exc())
            return np.zeros(points.shape[0], dtype=np.int32)  # 오류 시 기본값 반환

    def publish_filtered_pointcloud(self, header, points):
        try:
            # 필드 정의 (x, y, z, intensity)
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]

            # 원래 header의 frame_id 확인
            if not header.frame_id:
                # frame_id가 없으면 기본값 설정
                header.frame_id = "os_lidar"

            # 포인트를 리스트로 변환
            point_list = points.tolist()

            # PointCloud2 메시지 생성 및 발행
            cloud_msg = pc2.create_cloud(header, fields, point_list)
            self.pub.publish(cloud_msg)
            rospy.loginfo("필터링된 포인트 클라우드 발행 완료")
        except Exception as e:
            rospy.logerr(f"포인트 클라우드 발행 중 오류 발생: {e}")
            rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    try:
        frnet_seg = FRNetSegmentation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
