import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MouseClickNode(Node):
    def __init__(self):
        super().__init__('mouse_click_node')
        # camera1의 원본 영상을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            '/bev_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.points = []
        
        # 윈도우 창 이름 설정 및 마우스 클릭 이벤트 연결
        self.window_name = "Click 4 Points (Press 'ESC' to exit, 'c' to clear)"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        self.get_logger().info("좌표 추출기를 실행합니다. 바닥의 점 4개를 순서대로 클릭하세요!")
        self.get_logger().info("순서 추천: 1.왼쪽아래 -> 2.오른쪽아래 -> 3.오른쪽위 -> 4.왼쪽위")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.points) < 4:
                self.points.append([x, y])
                self.get_logger().info(f"{len(self.points)}번째 점 찍힘: [{x}, {y}]")
                
                if len(self.points) == 4:
                    self.get_logger().info(f"🎉 4개 좌표 수집 완료! 복사해서 사용하세요:\n{self.points}")

    def image_callback(self, msg):
        # ROS 이미지를 OpenCV용으로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 클릭한 곳에 빨간색 점 그리기
        for pt in self.points:
            cv2.circle(cv_image, tuple(pt), 5, (0, 0, 255), -1)
            
        cv2.imshow(self.window_name, cv_image)
        
        key = cv2.waitKey(1)
        if key == 27: # ESC 키 누르면 종료
            rclpy.shutdown()
        elif key == ord('c'): # c 키 누르면 점 초기화
            self.points = []
            self.get_logger().info("좌표가 초기화되었습니다. 다시 클릭하세요.")

def main(args=None):
    rclpy.init(args=args)
    node = MouseClickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()