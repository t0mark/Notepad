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
        
        # 시간 측정을 위한 변수들 추가
        self.processing_times = []
        self.max_times_to_keep = 10  # 최근 10개 프레임의 평균을 계산

        # 파라미터 로드
        self.config_path = rospy.get_param('~config', '/home/tomark/catkin_ws/src/frnet_ros/configs/frnet/frnet-semantickitti_seg.py')
        self.checkpoint_path = rospy.get_param('~checkpoint', '/home/tomark/catkin_ws/src/frnet_ros/checkpoints/frnet_semantickitti.pth')
        self.device = rospy.get_param('~device', 'cuda:0')
        
        # 디버깅 모드 및 설정
        self.debug = rospy.get_param('~debug', True)
        
        # 모델 초기화
        rospy.loginfo("모델 초기화 중...")
        try:
            self.model = init_model(self.config_path, self.checkpoint_path, device=self.device)
            rospy.loginfo("모델 초기화 완료")
        except Exception as e:
            rospy.logerr(f"모델 초기화 실패: {e}")
            rospy.logerr(traceback.format_exc())
            sys.exit(1)
        
        # 새로운 클래스 매핑 생성
        self.new_mapping = self.create_new_mapping()
        
        # 구독자 및 발행자 설정
        self.sub = rospy.Subscriber('/ouster/points', PointCloud2, self.pointcloud_callback, queue_size=1)
        self.pub = rospy.Publisher('/segmented_pc2', PointCloud2, queue_size=1)
        
        rospy.loginfo("FRNet 세그먼테이션 노드가 준비되었습니다")
    
    def create_new_mapping(self):
        """FRNet 모델의 출력 클래스(0-19)를 새로운 5개 클래스로 매핑합니다."""
        # 기본적으로 모든 클래스를 4(unlabeled)로 설정
        frnet_to_new = {i: 4 for i in range(20)}
        
        # 자동차(car) 클래스로 매핑 - 0
        frnet_to_new[0] = 0  # car
        frnet_to_new[3] = 0  # truck
        
        # 기타 차량(other-vehicle) 클래스로 매핑 - 1
        frnet_to_new[1] = 1  # bicycle
        frnet_to_new[2] = 1  # motorcycle
        frnet_to_new[4] = 1  # bus 
        frnet_to_new[5] = 1  # preson 
        frnet_to_new[6] = 1  # bicyclist
        frnet_to_new[7] = 1  # motorcyclist
        
        # 도로(road) 클래스로 매핑 - 2
        frnet_to_new[8] = 2  # road
        frnet_to_new[9] = 2  # parking
        
        # 인도(sidewalk) 클래스로 매핑 - 3
        frnet_to_new[10] = 3  # sidewalk
        
        return frnet_to_new
    
    def apply_new_mapping(self, original_labels):
        """원본 레이블에 새 매핑을 적용합니다."""
        new_labels = np.copy(original_labels)
        
        # 고유한 레이블 값에 대해 매핑 적용
        unique_labels = np.unique(original_labels)
        for label in unique_labels:
            mask = (original_labels == label)
            new_labels[mask] = self.new_mapping.get(label, 4)  # 기본값은 4(unlabeled)
        
        return new_labels
    
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
        # 처리 시작 시간 기록
        start_time = rospy.Time.now()
        rospy.loginfo("포인트 클라우드 데이터 수신")
        
        try:
            # 세부 단계별 시간 측정을 위한 변수들
            extraction_start = rospy.Time.now()
            
            # PointCloud2 메시지에서 직접 포인트 데이터 추출
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                points.append([p[0], p[1], p[2], p[3]])
            
            if not points:
                rospy.logwarn("포인트 클라우드가 비어 있습니다.")
                return
                    
            points = np.array(points, dtype=np.float32)
            
            extraction_time = (rospy.Time.now() - extraction_start).to_sec()
            
            # 디버깅 정보 출력
            if self.debug:
                rospy.loginfo(f"포인트 클라우드 형태: {points.shape}, 타입: {points.dtype}")
                rospy.loginfo(f"포인트 범위 - X: [{np.min(points[:,0]):.2f}, {np.max(points[:,0]):.2f}], "
                            f"Y: [{np.min(points[:,1]):.2f}, {np.max(points[:,1]):.2f}], "
                            f"Z: [{np.min(points[:,2]):.2f}, {np.max(points[:,2]):.2f}]")
            
            # 전처리 시작 시간
            preprocess_start = rospy.Time.now()
            
            # 이상치 제거 (너무 멀리 있는 포인트 제거)
            distance = np.sqrt(np.sum(points[:, :3] ** 2, axis=1))
            valid_indices = distance < 100.0  # 100m 범위 내 포인트만 사용
            points = points[valid_indices]
            
            # SemanticKITTI 형식으로 변환
            points = self.convert_to_semantickitti_format(points)
            
            # 전체 포인트 클라우드 사용
            if self.debug:
                rospy.loginfo(f"전체 포인트 클라우드 사용: {points.shape[0]} 포인트")
                
            preprocess_time = (rospy.Time.now() - preprocess_start).to_sec()
            
            # 추론 시작 시간
            inference_start = rospy.Time.now()
            
            # 포인트 클라우드 추론
            pred_labels = self.infer_pointcloud(points)
            
            inference_time = (rospy.Time.now() - inference_start).to_sec()
            
            # 발행 시작 시간
            publish_start = rospy.Time.now()
            
            # 결과를 ROS 메시지로 변환하여 발행
            self.publish_segmented_pointcloud(msg.header, points, pred_labels)
            
            publish_time = (rospy.Time.now() - publish_start).to_sec()
            
            # 전체 처리 시간 계산
            end_time = rospy.Time.now()
            processing_time = (end_time - start_time).to_sec()
            
            # 처리 시간 기록
            self.processing_times.append(processing_time)
            if len(self.processing_times) > self.max_times_to_keep:
                self.processing_times.pop(0)
            
            # 평균 처리 시간 및 주사율 계산
            avg_time = sum(self.processing_times) / len(self.processing_times)
            current_rate = 1.0 / processing_time
            avg_rate = 1.0 / avg_time
            
            rospy.loginfo("성능 측정:")
            rospy.loginfo(f"  데이터 추출 시간: {extraction_time:.4f}초")
            rospy.loginfo(f"  전처리 시간: {preprocess_time:.4f}초")
            rospy.loginfo(f"  추론 시간: {inference_time:.4f}초")
            rospy.loginfo(f"  발행 시간: {publish_time:.4f}초")
            rospy.loginfo(f"  총 처리 시간: {processing_time:.4f}초 (주사율: {current_rate:.2f}Hz)")
            rospy.loginfo(f"  평균 처리 시간: {avg_time:.4f}초 (평균 주사율: {avg_rate:.2f}Hz)")
            
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
                
                # 원래 20개 클래스로 예측된 레이블
                original_pred_labels = results[0].pred_pts_seg.pts_semantic_mask.cpu().numpy()
                
                # 새로운 5개 클래스로 매핑 적용
                new_pred_labels = self.apply_new_mapping(original_pred_labels)
                
                # 디버깅 정보
                if self.debug:
                    unique_labels, counts = np.unique(new_pred_labels, return_counts=True)
                    rospy.loginfo(f"세그먼테이션 결과 - 레이블 분포: {dict(zip(unique_labels, counts))}")
                
                return new_pred_labels
        except Exception as e:
            rospy.logerr(f"추론 과정 중 예외 발생: {e}")
            rospy.logerr(traceback.format_exc())
            return np.zeros(points.shape[0], dtype=np.int32)  # 오류 시 기본값 반환
    
    def publish_segmented_pointcloud(self, header, points, labels):
        try:
            # 라벨별 색상 정의 (RGB 형식, 0~255)
            color_map = {
                0: [255, 0, 0],    # 빨강 - 자동차
                1: [0, 255, 0],    # 초록 - 기타 차량
                2: [0, 0, 255],    # 파랑 - 도로
                3: [0, 255, 255],  # 노랑 - 인도
                4: [255, 255, 255] # 회색 - 기타
            }
            
            # 포인트 클라우드에 색상 정보 추가
            colored_points = []
            for i, point in enumerate(points):
                x, y, z, intensity = point
                label = int(labels[i])
                
                # RGB 색상을 float32로 변환 (RViz RGB8 포맷)
                r, g, b = color_map.get(label, [100, 100, 100])
                rgb = struct.unpack('f', struct.pack('BBBx', r, g, b))[0]
                
                colored_points.append([x, y, z, rgb])
            
            # 필드 정의
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            # 원래 header의 frame_id 확인
            if not header.frame_id:
                # frame_id가 없으면 기본값 설정
                header.frame_id = "ouster"  # 또는 적절한 프레임 ID로 변경
                
            # PointCloud2 메시지 생성 및 발행
            cloud_msg = pc2.create_cloud(header, fields, colored_points)
            self.pub.publish(cloud_msg)
            rospy.loginfo("세그먼테이션된 포인트 클라우드 발행 완료")
        except Exception as e:
            rospy.logerr(f"포인트 클라우드 발행 중 오류 발생: {e}")
            rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    try:
        frnet_seg = FRNetSegmentation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass