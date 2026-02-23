import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class LaneYOLONode(Node):
    def __init__(self):
        super().__init__('lane_yolo_node')
        
        # --- 🛠️ 파라미터 설정 ---
        self.real_width_m = 2.0  # 실제 도로의 가로 길이 (미터)
        self.real_height_m = 5.0 # 실제 도로의 세로 길이 (미터)
        self.bev_width = 320
        self.bev_height = 240
        
        # --- 🛠️ YOLOv11 모델 로드 ---
        # 본인의 best.pt 경로로 수정하세요
        model_path = '/home/woonggook/workspace/5challenge/best.pt'
        self.model = YOLO(model_path)
        
        # --- 🚀 ROS 2 통신 설정 ---
        # BEV 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_bev', 
            self.listener_callback,
            10)
        
        # 결과 이미지(Segmentation) 발행
        self.seg_publisher = self.create_publisher(Image, '/camera1/image_seg', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Lane YOLO Node started.')

    def listener_callback(self, msg):
        try:
            # 1. ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. YOLOv11 추론
            # 속도 제한을 위해 imgsz를 낮추고 conf를 조절합니다.
            results = self.model(cv_image, conf=0.4, imgsz=320, verbose=False)
            
            # 3. 결과 시각화
            annotated_frame = results[0].plot() 
            
            # 4. 🚀 웨이포인트(Waypoint) 계산
            self.calculate_waypoints(results)
            
            # 5. 결과 발행
            self.seg_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

    def calculate_waypoints(self, results):
        if results[0].masks is None or len(results[0].masks) == 0:
            self.get_logger().warn('No lanes detected!')
            return

        # 1. 마스크 데이터 가져오기
        masks = results[0].masks.xy  # List of polygon points
        
        left_lane = []
        right_lane = []

        # 2. 픽셀 좌표 기준으로 좌우 구분
        for mask in masks:
            mean_x = np.mean(mask[:, 0])
            if mean_x < self.bev_width / 2:
                left_lane.append(mask)
            else:
                right_lane.append(mask)

        # 3. 좌우 차선이 모두 있어야 중심선 계산 가능
        if not left_lane or not right_lane:
            self.get_logger().warn('Need both left and right lanes!')
            return

        # 4. 좌우 차선에서 각각 대표 포인트(중앙값) 계산
        # (더 정교하게 하려면 polyfit 사용 필요)
        l_line = max(left_lane, key=lambda x: len(x))
        r_line = min(right_lane, key=lambda x: len(x))
        
        # 5. 대표 웨이포인트 3개 선정 (가까운곳, 중간, 먼곳)
        for i in range(3):
            y_ratio = 0.8 - i * 0.3 # 0.8, 0.5, 0.2
            
            # 해당 높이(y)에서 좌우 차선의 x좌표 보간(예시: 단순 평균)
            # 💡 주의: 이 부분은 실제 차선 형태에 따라 보간법 구현이 필요합니다.
            
            # 임시로 YOLO가 찾아낸 마스크의 y범위 내에서 중심점 계산
            p_y = int(self.bev_height * y_ratio)
            
            # 픽셀 -> 실제 미터 좌표 변환
            # 여기서는 좌우 평균을 내는 논리가 필요합니다.
            p_x = int(self.bev_width / 2) # 👈 핵심: 좌/우 차선 마스크 좌표 기반 중심점으로 변경해야 함
            
            real_x, real_y = self.pixel_to_meter(p_x, p_y)
            self.get_logger().info(f'Waypoint {i}: X={real_x:.2f}m, Y={real_y:.2f}m')

    def pixel_to_meter(self, px, py):
        # BEV 픽셀 좌표를 0~1 사이의 비율로 변환
        ratio_x = px / self.bev_width
        ratio_y = py / self.bev_height
        
        # 실제 미터 단위로 변환 (카메라 위치에 따라 조정)
        real_x = (ratio_x - 0.5) * self.real_width_m  # 중앙이 0
        real_y = (1.0 - ratio_y) * self.real_height_m # 아래쪽이 0m (차량 앞)
        
        return real_x, real_y

def main(args=None):
    rclpy.init(args=args)
    node = LaneYOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()