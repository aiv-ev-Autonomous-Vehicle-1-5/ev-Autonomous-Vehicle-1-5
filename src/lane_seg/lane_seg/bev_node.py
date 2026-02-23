#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BEVTransformNode(Node):
    def __init__(self):
        super().__init__('bev_transform_node')
        
        self.bridge = CvBridge()
        
        # 1. 구독(Subscriber)과 발행(Publisher) 설정
        # 주의: '/image_raw' 부분을 실제 사용하는 카메라 토픽 이름으로 변경해야 합니다.
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw', 
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(Image, '/bev_image', 10)
        
        self.get_logger().info("BEV 변환 노드 초기화 중... 수학 연산 1회 수행")

        # =========================================================
        # 2. [정석] 노드 실행 시 LUT(지시서)를 딱 한 번만 계산하여 메모리에 저장
        # =========================================================
        self.camera_matrix = np.array([
            [484.45878, 0.0, 291.5055],
            [0.0, 482.5031, 222.66382],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        # 우리가 구한 완벽한 Extrinsic 파라미터
        tvec = np.array([[0.04156545], [1.17812103], [0.46439937]], dtype=np.float32)
        rmat = np.array([
            [-0.99990688,  0.0106132,  -0.00857803],
            [-0.01221857, -0.41637236,  0.90911207],
            [ 0.00607693,  0.90913223,  0.41646327]
        ], dtype=np.float32)

        extrinsic = np.hstack((rmat, tvec))

        # BEV 도화지 물리적 범위 (미터 단위)
        world_y_min, world_y_max = 1.0, 5.0  # 전방 1m ~ 5m
        world_x_min, world_x_max = -2.0, 2.0 # 좌우 -2m ~ 2m
        interval = 0.02 # 1픽셀 = 2cm 해상도

        self.output_height = int(np.ceil((world_y_max - world_y_min) / interval))
        self.output_width = int(np.ceil((world_x_max - world_x_min) / interval))

        self.map_x = np.zeros((self.output_height, self.output_width), dtype=np.float32)
        self.map_y = np.zeros((self.output_height, self.output_width), dtype=np.float32)

        world_y_coords = np.arange(world_y_max, world_y_min, -interval) 
        world_x_coords = np.arange(world_x_max, world_x_min, -interval) 

        for i, world_y in enumerate(world_y_coords):
            for j, world_x in enumerate(world_x_coords):
                # [X(좌우), Y(전방), Z(바닥), 1]
                world_coord = np.array([world_x, world_y, 0, 1]).reshape(4, 1)
                
                camera_coord = extrinsic @ world_coord
                uv_coord = self.camera_matrix @ camera_coord
                
                u = uv_coord[0, 0] / uv_coord[2, 0]
                v = uv_coord[1, 0] / uv_coord[2, 0]
                
                self.map_x[i, j] = u
                self.map_y[i, j] = v

        self.get_logger().info(f"✅ LUT 생성 완료 (크기: {self.output_width}x{self.output_height}) - 실시간 변환 대기 중!")

    def image_callback(self, msg):
        try:
            # 1. ROS 2 Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. [초고속 변환] 미리 만들어둔 LUT 지시서를 보고 픽셀만 재배치
            bev_image = cv2.remap(cv_image, self.map_x, self.map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            
            # 3. OpenCV 이미지를 다시 ROS 2 Image 메시지로 변환하여 발행
            bev_msg = self.bridge.cv2_to_imgmsg(bev_image, encoding='bgr8')
            bev_msg.header = msg.header
            self.publisher.publish(bev_msg)
            
        except Exception as e:
            self.get_logger().error(f"이미지 변환 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BEVTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()