#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import torch
from ultralytics import YOLO

from ev_msgs.msg import LaneBoundary, LaneBoundaryArray

class YoloDbSegNode(Node):
    def __init__(self):
        super().__init__('yolo_db_seg_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/bev_image', self.image_callback, 10)

        self.img_publisher = self.create_publisher(Image, '/yolo_db_seg_image', 10)

        # Planning 노드와 동일한 Best Effort QoS (depth=1)
        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.coord_publisher = self.create_publisher(
            LaneBoundaryArray, '/perception/lane_boundaries', qos_be)

        # BEV 좌표 변환 (bev_cali.py / bev_node.py 기준 동기화)
        self.car_x_px = 226
        self.car_y_px = 419
        self.interval = 0.01

        # CUDA 디바이스 설정
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"사용 디바이스: {self.device}")

        self.get_logger().info("YOLO 모델 로딩 중...")
        model_path = os.path.expanduser('~/ev-Autonomous-Vehicle-1-5/perception/src/camera/lane_seg/config/fourth_best.pt')

        try:
            self.model = YOLO(model_path)
            self.model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            raise e

        self.get_logger().info("YOLO DBSCAN 노드 가동!")

    def image_callback(self, msg):
        try:
            bev_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = bev_frame.shape[:2]

            results = self.model.predict(bev_frame, conf=0.7, imgsz=320, half=True, verbose=False, device=self.device)
            result = results[0]

            color_mask = np.zeros_like(bev_frame)

            # LaneBoundaryArray 메시지 생성
            lane_arr_msg = LaneBoundaryArray()
            lane_arr_msg.header = Header()
            lane_arr_msg.header.stamp = self.get_clock().now().to_msg()
            lane_arr_msg.header.frame_id = 'base_link'

            center_points = []

            if result.masks is not None:
                # 모든 마스크를 하나로 합침
                combined_mask = np.zeros((h, w), dtype=np.float32)
                masks = result.masks.data.cpu().numpy()
                for mask in masks:
                    mask_resized = cv2.resize(mask, (w, h))
                    combined_mask = np.maximum(combined_mask, mask_resized)

                color_mask[combined_mask > 0.5] = [255, 0, 0]

                # 평균 검출 신뢰도 계산
                avg_conf = float(result.boxes.conf.mean()) if result.boxes is not None and len(result.boxes) > 0 else 1.0

                # 마스크 픽셀 추출
                indices = np.where(combined_mask > 0.5)
                rows, cols = indices[0], indices[1]

                if len(rows) > 0:
                    # 1D gap-sort 클러스터링 (sklearn DBSCAN 대체, eps=30 min_samples=50)
                    sorted_idx = np.argsort(cols)
                    sorted_cols = cols[sorted_idx]
                    sorted_rows = rows[sorted_idx]
                    gaps = np.where(np.diff(sorted_cols) > 30)[0] + 1
                    splits_c = np.split(sorted_cols, gaps)
                    splits_r = np.split(sorted_rows, gaps)

                    for cluster_cols, cluster_rows in zip(splits_c, splits_r):
                        if len(cluster_cols) < 50:
                            continue

                        # LaneBoundary 메시지 생성 (클러스터당 하나)
                        boundary = LaneBoundary()
                        boundary.header = lane_arr_msg.header
                        boundary.confidence = avg_conf

                        # 행별 중심점 벡터화 추출
                        unique_rows = np.unique(cluster_rows)
                        sampled = unique_rows[::10]
                        boundary_points = []
                        for row in sampled:
                            center_x = int(np.mean(cluster_cols[cluster_rows == row]))
                            center_points.append((center_x, row))

                            pt = Point()
                            # 차량 좌표계: x=전방(종방향), y=좌우(횡방향)
                            pt.x = float((self.car_y_px - row) * self.interval)
                            pt.y = float((self.car_x_px - center_x) * self.interval)
                            pt.z = 0.0
                            boundary_points.append(pt)

                        # 차량에서 가까운 점부터 정렬 (x 오름차순)
                        boundary_points.sort(key=lambda p: p.x)
                        boundary.points = boundary_points
                        lane_arr_msg.boundaries.append(boundary)

            self.coord_publisher.publish(lane_arr_msg)

            # 시각화
            final_result = cv2.addWeighted(bev_frame, 1, color_mask, 0.5, 0)

            if result.masks is not None:
                for (cx, cy) in center_points:
                    cv2.circle(final_result, (cx, cy), 3, (0, 0, 255), -1)

            if result.boxes is not None:
                for box in result.boxes.data.cpu().numpy():
                    conf, x1, y1 = box[4], int(box[0]), int(box[1])
                    cv2.putText(final_result, f"{conf*100:.1f}%", (x1, max(30, y1-10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            img_msg = self.bridge.cv2_to_imgmsg(final_result, encoding='bgr8')
            img_msg.header = msg.header
            self.img_publisher.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDbSegNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__': main()
