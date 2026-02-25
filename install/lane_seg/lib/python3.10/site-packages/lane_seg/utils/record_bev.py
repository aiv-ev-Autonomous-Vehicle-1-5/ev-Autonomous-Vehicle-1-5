import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')
        
        # /bev_image 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/bev_image',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        # 동영상 저장 설정 (처음엔 비워둠, 첫 프레임이 들어오면 해상도 맞춰서 시작)
        self.out = None
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID') # avi 포맷 코덱
        self.video_name = 'track_record.avi'
        
        self.get_logger().info('⏳ 카메라 켜지기를 기다리는 중... (/bev_image 토픽 대기)')

    def image_callback(self, msg):
        # ROS 2 이미지 -> OpenCV 이미지 변환
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # 첫 프레임이 들어왔을 때 딱 한 번 동영상 파일 생성 (해상도 자동 맞춤)
        if self.out is None:
            height, width, _ = cv_img.shape
            # 30.0 은 FPS(초당 프레임 수)입니다.
            self.out = cv2.VideoWriter(self.video_name, self.fourcc, 30.0, (width, height))
            self.get_logger().info(f'🔴 녹화 시작! (해상도: {width}x{height})')
            self.get_logger().info('⏹️ 녹화를 끝내려면 터미널에서 Ctrl+C 를 누르세요.')

        # 화면을 동영상 파일에 한 장씩 기록
        self.out.write(cv_img)
        
        # 녹화가 잘 되고 있는지 모니터에 살짝 띄워줌
        cv2.imshow('Recording BEV...', cv_img)
        cv2.waitKey(1)

    def destroy_node(self):
        # 노드가 꺼질 때 동영상 파일 안전하게 닫기
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
        node.get_logger().info(f'\n✅ 녹화 완료! 현재 폴더에 {node.video_name} 파일이 저장되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()