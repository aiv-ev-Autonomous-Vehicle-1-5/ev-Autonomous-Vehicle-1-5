import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime  # 현재 시간을 가져오기 위한 라이브러리 추가!

class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/bev_image',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        # --- 📂 바탕화면 경로 및 자동 네이밍 설정 ---
        home_dir = os.path.expanduser('~')
        target_dir = os.path.join(home_dir, 'Desktop', 'test_videos')
        os.makedirs(target_dir, exist_ok=True)
        
        # 실행하는 순간의 시간을 '20260225_145830' 형식으로 가져옵니다.
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 파일 이름에 시간을 붙여서 겹치지 않게 만듭니다!
        file_name = f'track_record_{current_time}.avi'
        self.video_path = os.path.join(target_dir, file_name)
        # -----------------------------------------------
        
        self.out = None
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        
        self.get_logger().info('⏳ 카메라 켜지기를 기다리는 중... (/bev_image 토픽 대기)')

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        if self.out is None:
            height, width, _ = cv_img.shape
            self.out = cv2.VideoWriter(self.video_path, self.fourcc, 30.0, (width, height))
            
            self.get_logger().info(f'🔴 녹화 시작! (해상도: {width}x{height})')
            self.get_logger().info(f'📂 저장 위치: {self.video_path}')
            self.get_logger().info('⏹️ 녹화를 끝내려면 터미널에서 Ctrl+C 를 누르세요.')

        self.out.write(cv_img)
        cv2.imshow('Recording BEV...', cv_img)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.out is not None:
            self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'\n✅ 녹화 완료! {node.video_path} 에 안전하게 저장되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()