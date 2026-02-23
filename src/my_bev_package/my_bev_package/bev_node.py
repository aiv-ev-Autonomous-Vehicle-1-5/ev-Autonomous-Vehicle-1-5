import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 왜곡 보정 image_proc 하기
# ros2 run image_proc image_proc --ros-args -r __ns:=/camera1 -r image:=image_raw

class BEVConverter(Node):
    def __init__(self):
        super().__init__('bev_node')
        # 보정된 영상(rect)을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_rect', 
            self.listener_callback,
            10)
        # BEV 영상을 발행합니다.
        self.publisher = self.create_publisher(Image, '/camera1/image_bev', 10)
        self.bridge = CvBridge()
        
        # --- 🛠️ BEV 변환 행렬 설정 (여기 숫자를 나중에 튜닝해야 함) ---
        # 원본 영상(image_rect)에서의 4개 점
        self.src_pts = np.float32([
            [270, 294], # 좌상
            [416, 305], # 우상
            [500, 461], # 우하
            [177, 433]    # 좌하
        ])
        
        # 변환 후 영상(image_bev)에서의 4개 점 (위에서 본 직사각형 모양)
        self.dst_pts = np.float32([
            [0, 0],
            [640, 0],
            [640, 480],
            [0, 480]
        ])
        
        # 변환 행렬 계산
        self.M = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)

    def listener_callback(self, msg):
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # BEV 변환(Perspective Warp) 적용
            bev_image = cv2.warpPerspective(cv_image, self.M, (640, 480))
            
            # 다시 ROS 메시지로 변환하여 발행
            bev_msg = self.bridge.cv2_to_imgmsg(bev_image, encoding='bgr8')
            self.publisher.publish(bev_msg)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BEVConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()