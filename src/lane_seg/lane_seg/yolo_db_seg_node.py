#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO
from sklearn.cluster import DBSCAN

from lane_seg_msgs.msg import LaneCoords

class YoloDbSegNode(Node):
    def __init__(self):
        super().__init__('yolo_db_seg_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/bev_image', self.image_callback, 10)

        self.img_publisher = self.create_publisher(Image, '/yolo_db_seg_image', 10)
        self.coord_publisher = self.create_publisher(LaneCoords, '/lane_db_coordinates', 10)

        # BEV 좌표 변환 (bev_cali.py / bev_node.py 기준 동기화)
        self.car_x_px = 226
        self.car_y_px = 419
        self.interval = 0.01

        self.get_logger().info("YOLO 모델 로딩 중...")
        model_path = os.path.expanduser('~/ev_ws/src/lane_seg/config/second_best.pt')

        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            raise e

        self.get_logger().info("YOLO DBSCAN 노드 가동!")

    def image_callback(self, msg):
        try:
            bev_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = bev_frame.shape[:2]

            results = self.model.predict(bev_frame, conf=0.7, imgsz=320, verbose=False)
            result = results[0]

            color_mask = np.zeros_like(bev_frame)
            lane_msg = LaneCoords()

            if result.masks is not None:
                # 모든 마스크를 하나로 합침
                combined_mask = np.zeros((h, w), dtype=np.float32)
                masks = result.masks.data.cpu().numpy()
                for mask in masks:
                    mask_resized = cv2.resize(mask, (w, h))
                    combined_mask = np.maximum(combined_mask, mask_resized)

                color_mask[combined_mask > 0.5] = [255, 0, 0]

                # 마스크 픽셀 추출
                indices = np.where(combined_mask > 0.5)
                rows, cols = indices[0], indices[1]

                center_points = []

                if len(rows) > 0:
                    # DBSCAN 클러스터링 (x값 기준으로 그룹 분리)
                    points = cols.reshape(-1, 1)
                    db = DBSCAN(eps=30, min_samples=50).fit(points)
                    labels = db.labels_

                    # 유효한 클러스터만 (노이즈 label=-1 제거)
                    valid_labels = set(labels[labels >= 0])

                    for label in valid_labels:
                        cluster_mask = labels == label
                        cluster_rows = rows[cluster_mask]
                        cluster_cols = cols[cluster_mask]

                        # 행별로 중심점 추출
                        unique_rows = np.unique(cluster_rows)
                        for row in unique_rows[::10]:
                            cols_in_row = cluster_cols[cluster_rows == row]
                            center_x = int(np.mean(cols_in_row))
                            center_points.append((center_x, row))

                            real_x = float((self.car_y_px - row) * self.interval)
                            real_y = float((self.car_x_px - center_x) * self.interval)
                            lane_msg.line_x.append(real_x)
                            lane_msg.line_y.append(real_y)

            self.coord_publisher.publish(lane_msg)

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
