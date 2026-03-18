#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class BEVLutPublisher(Node):
    def __init__(self):
        super().__init__('bev_lut_node')
        self.bridge = CvBridge()
        
        # 1. 구독 및 발행 설정
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',  # 실제 사용하는 카메라 토픽으로 수정하세요
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/bev_image', 10)
        
        self.get_logger().info("⏳ BEV 지시서(LUT) 생성 중...")

        # 2. 카메라 파라미터 세팅 (렌즈 왜곡 펴기용)
        self.K = np.array([
            [484.45878, 0.0, 291.5055],
            [0.0, 482.5031, 222.66382],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.D = np.array([0.162875, -0.287212, 0.000692, -0.000678, 0.0])

        # 3. 캘리브레이션으로 구한 NPY 파일 로드
        # 3. 캘리브레이션으로 구한 NPY 파일 로드
        npy_path = os.path.expanduser('~/ev-Autonomous-Vehicle-1-5/perception/src/camera/lane_seg/config/bev_matrix.npy')
        try:
            self.H_matrix = np.load(npy_path)
        except Exception as e:
            self.get_logger().error(f"❌ NPY 파일을 찾을 수 없습니다: {e}")
            raise e

        # 4. 출력될 BEV 이미지의 해상도 설정 (캘리브레이션 시 설정한 크기와 동일하게)
        self.bev_w = 400
        self.bev_h = 400

        # =========================================================
        # 5. [핵심] Homography 행렬(H)을 이용해 LUT(map_x, map_y) 생성
        # 목적지(BEV)의 모든 픽셀 좌표에 대해 원본 이미지의 어느 픽셀을 가져올지 역산(H_inv)
        # =========================================================
        H_inv = np.linalg.inv(self.H_matrix)
        
        # BEV 도화지의 모든 (x, y) 픽셀 좌표 생성
        x, y = np.meshgrid(np.arange(self.bev_w), np.arange(self.bev_h))
        ones = np.ones_like(x)
        
        # (3, bev_w * bev_h) 형태로 변환하여 한 번에 행렬 곱셈 연산
        bev_coords = np.vstack((x.flatten(), y.flatten(), ones.flatten()))
        
        # 원본 이미지의 픽셀 좌표로 역산
        cam_coords = H_inv @ bev_coords
        
        # Z값으로 나눠서 2D 좌표로 정규화 (w'로 나누기)
        u = cam_coords[0] / cam_coords[2]
        v = cam_coords[1] / cam_coords[2]
        
        # cv2.remap에서 사용할 수 있게 map_x, map_y로 형태 변환
        self.map_x = u.reshape(self.bev_h, self.bev_w).astype(np.float32)
        self.map_y = v.reshape(self.bev_h, self.bev_w).astype(np.float32)

        self.get_logger().info("✅ LUT(map_x, map_y) 매핑 완료! 초고속 BEV 발행 대기 중!")

    def image_callback(self, msg):
        try:
            # 1. ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. 렌즈 왜곡 펴기 (H 행렬이 왜곡 없는 상태를 기준으로 만들어졌으므로 필수!)
            undistorted_image = cv2.undistort(cv_image, self.K, self.D)
            
            # 3. 미리 만들어둔 LUT 지시서를 이용해 픽셀 순간이동 (초고속 탑뷰 변환)
            bev_image = cv2.remap(undistorted_image, self.map_x, self.map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            
            # BEV 결과 확인용
            #cv2.imshow("BEV LUT View", bev_image)
            #cv2.waitKey(1)

            # 4. BEV 이미지를 ROS 토픽으로 발행
            bev_msg = self.bridge.cv2_to_imgmsg(bev_image, encoding='bgr8')
            bev_msg.header = msg.header
            self.publisher.publish(bev_msg)
            
        except Exception as e:
            self.get_logger().error(f"이미지 처리 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BEVLutPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()