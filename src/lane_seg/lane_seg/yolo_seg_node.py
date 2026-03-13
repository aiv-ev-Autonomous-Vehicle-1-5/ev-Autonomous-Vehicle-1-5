#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO

# 🎯 방금 우리가 직접 만든 커스텀 메시지 임포트!
from lane_seg_msgs.msg import LaneCoords  # 🎯 새로 만드신 패키지 이름으로 변경! 

class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image, '/bev_image', self.image_callback, 10)
        
        self.img_publisher = self.create_publisher(Image, '/yolo_seg_image', 10)
        
        # 🎯 커스텀 메시지 타입으로 토픽 발행
        self.coord_publisher = self.create_publisher(LaneCoords, '/lane_coordinates', 10)
        
        # BEV 좌표 변환 (bev_cali.py / bev_node.py 기준 동기화)
        # BEV: 400x400px, 1px = 0.01m (px_per_meter=100)
        # 차 앞범퍼 위치: x=226px (H변환 계산), y=419px (카메라바닥373 + 범퍼거리46cm)
        self.car_x_px = 226
        self.car_y_px = 419   # 373 + 46 (카메라→범퍼 46cm)
        self.interval = 0.01  # 1px = 0.01m

        self.get_logger().info("⏳ YOLO 모델 로딩 중...")
        model_path = os.path.expanduser('~/ev_ws/src/lane_seg/config/second_best.pt')
        
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"❌ YOLO 모델 로드 실패: {e}")
            raise e

        self.get_logger().info("🚀 YOLO 노드 (커스텀 메시지 LaneCoords 적용) 가동!")

    def image_callback(self, msg):
        try:
            bev_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = bev_frame.shape[:2]

            results = self.model.predict(bev_frame, conf=0.7, imgsz=320, verbose=False)
            result = results[0]
            
            color_mask = np.zeros_like(bev_frame)
            
            # 🎯 우리가 만든 커스텀 메시지 빈 껍데기 소환
            lane_msg = LaneCoords()

            if result.masks is not None:
                # 모든 마스크를 하나로 합침
                combined_mask = np.zeros((h, w), dtype=np.float32)
                masks = result.masks.data.cpu().numpy()
                for mask in masks:
                    mask_resized = cv2.resize(mask, (w, h))
                    combined_mask = np.maximum(combined_mask, mask_resized)

                color_mask[combined_mask > 0.5] = [255, 0, 0]

                # 행(row)별로 중심점 추출
                center_points = []  # 시각화용 픽셀 좌표 저장
                unique_rows = np.where(np.any(combined_mask > 0.5, axis=1))[0]
                for row in unique_rows[::10]:  # 10행 간격
                    cols_in_row = np.where(combined_mask[row] > 0.5)[0]
                    if len(cols_in_row) == 0:
                        continue

                    # x값 정렬 후 100px 이상 차이나는 구간에서 그룹 분리
                    sorted_cols = np.sort(cols_in_row)
                    gaps = np.where(np.diff(sorted_cols) > 100)[0]
                    groups = np.split(sorted_cols, gaps + 1)

                    for group in groups:
                        center_x = int(np.mean(group))
                        center_points.append((center_x, row))
                        # 차량 좌표계: x=전방(종방향), y=좌우(횡방향)
                        real_x = float((self.car_y_px - row) * self.interval)       # 전방 거리
                        real_y = float((self.car_x_px - center_x) * self.interval)  # 좌(+) / 우(-)
                        lane_msg.line_x.append(real_x)
                        lane_msg.line_y.append(real_y)

            # 4. 꽉 찬 커스텀 메시지 발행
            self.coord_publisher.publish(lane_msg)

            # 5. 시각화 처리
            final_result = cv2.addWeighted(bev_frame, 1, color_mask, 0.5, 0)

            # 중심점 빨간 점으로 표시
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
    node = YoloSegNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__': main()