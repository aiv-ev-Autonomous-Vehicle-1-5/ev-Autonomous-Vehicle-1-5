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

class YoloInstanceSegNode(Node):
    def __init__(self):
        super().__init__('yolo_instance_seg_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/bev_image', self.image_callback, 10)

        self.img_publisher = self.create_publisher(Image, '/yolo_instance_seg_image', 10)

        # yolo_lane_cluster 노드로 전달 — Best Effort QoS (depth=1)
        qos_be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.coord_publisher = self.create_publisher(
            LaneBoundaryArray, '/perception/raw_lane_boundaries', qos_be)

        # BEV 좌표 변환 (bev_cali.py / bev_node.py 기준 동기화)
        self.car_x_px = 226
        self.car_y_px = 419
        self.interval = 0.01

        # CUDA 디바이스 설정
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"사용 디바이스: {self.device}")

        self.get_logger().info("YOLO 모델 로딩 중...")
        model_path = os.path.expanduser('~/ev-Autonomous-Vehicle-1-5/perception/src/camera/lane_seg/config/ninth_best.pt')

        try:
            self.model = YOLO(model_path)
            self.model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            raise e

        self.get_logger().info("YOLO 인스턴스 세그멘테이션 노드 가동!")

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
            lane_id_counter = 0

            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()
                confs = result.boxes.conf.cpu().numpy() if result.boxes is not None else np.ones(len(masks))

                # 각 마스크(인스턴스)를 개별 차선으로 처리 — 합치지 않음
                for mask_idx, mask in enumerate(masks):
                    mask_resized = cv2.resize(mask, (w, h))
                    binary_mask = (mask_resized > 0.5).astype(np.uint8)

                    # 시각화용 컬러 마스크 누적
                    color_mask[binary_mask > 0] = [255, 0, 0]

                    # 마스크 픽셀 추출
                    mask_rows, mask_cols = np.where(binary_mask > 0)

                    # 노이즈 필터링: 픽셀 수가 너무 적으면 무시
                    if len(mask_rows) < 500:
                        continue

                    # 해당 인스턴스의 신뢰도
                    conf = float(confs[mask_idx]) if mask_idx < len(confs) else 1.0

                    # LaneBoundary 메시지 생성 (인스턴스당 하나)
                    boundary = LaneBoundary()
                    boundary.header = lane_arr_msg.header
                    boundary.confidence = conf

                    # 행별 중심점 추출
                    unique_rows = np.unique(mask_rows)
                    sampled = unique_rows[::10]
                    boundary_points = []
                    for row in sampled:
                        row_pixels = np.sort(mask_cols[mask_rows == row])
                        if len(row_pixels) < 5:
                            continue
                        # 최대 연속구간 찾기 (gap > 5px이면 분리)
                        gaps = np.where(np.diff(row_pixels) > 5)[0] + 1
                        segments = np.split(row_pixels, gaps)
                        longest = max(segments, key=len)
                        center_x = int((longest[0] + longest[-1]) / 2)
                        center_points.append((center_x, row))

                        pt = Point()
                        # 차량 좌표계: x=전방(종방향), y=좌우(횡방향)
                        pt.x = float((self.car_y_px - row) * self.interval)
                        pt.y = float((self.car_x_px - center_x) * self.interval)
                        pt.z = 0.0
                        boundary_points.append(pt)

                    if len(boundary_points) < 5:
                        continue

                    # 차량에서 가까운 점부터 정렬 (x 오름차순)
                    boundary_points.sort(key=lambda p: p.x)
                    boundary.points = boundary_points
                    boundary.lane_id = lane_id_counter
                    lane_arr_msg.boundaries.append(boundary)
                    lane_id_counter += 1

            self.coord_publisher.publish(lane_arr_msg)

            # 시각화
            final_result = cv2.addWeighted(bev_frame, 1, color_mask, 0.5, 0)

            for (cx, cy) in center_points:
                cv2.circle(final_result, (cx, cy), 3, (0, 0, 255), -1)

            # 차선 ID 텍스트 표시
            for boundary in lane_arr_msg.boundaries:
                if boundary.points:
                    first_pt = boundary.points[0]
                    px = int(self.car_x_px - first_pt.y / self.interval)
                    py = int(self.car_y_px - first_pt.x / self.interval)
                    cv2.putText(final_result, f"ID:{boundary.lane_id}",
                                (px, max(20, py - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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
    node = YoloInstanceSegNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__': main()
