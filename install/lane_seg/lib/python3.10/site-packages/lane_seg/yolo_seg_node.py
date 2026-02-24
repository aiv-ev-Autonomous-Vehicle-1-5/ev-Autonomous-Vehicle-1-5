import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO

class YoloLaneSegNode(Node):
    def __init__(self):
        super().__init__('yolo_lane_seg_node')
        
        # 1. 구독자(Subscriber)와 발행자(Publisher) 설정
        self.subscription = self.create_subscription(
            Image,
            '/bev_image',  # 아까 쫙 펴놓은 BEV 도화지 토픽
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(Image, '/yolo_result_image', 10)
        
        # 🔥 아까 실수로 지워졌던 핵심 코드 복구!
        self.bridge = CvBridge()
        
        # 2. config 폴더에 있는 'first_best.pt' 모델 절대 경로로 로드
        home_dir = os.path.expanduser('~')
        model_path = os.path.join(home_dir, 'ev_ws', 'src', 'lane_seg', 'config', 'first_best.pt')
        
        self.get_logger().info(f'🚀 YOLO11 모델 로딩 중... 경로: {model_path}')
        
        if not os.path.exists(model_path):
            self.get_logger().error('❌ 모델 파일이 없습니다! 경로를 다시 확인해주세요.')
            return
            
        # 노트북 CPU 환경에 맞게 최적화 (버벅임 방지)
        self.model = YOLO(model_path)
        self.get_logger().info('✅ 모델 로딩 완료! 차선 인식을 시작합니다.')

    def image_callback(self, msg):
        # ROS 2 이미지 -> OpenCV 이미지로 변환
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        annotated_img = cv_img.copy()

        # 3. YOLO 추론 실행 (CPU 강제 지정, 로그 숨기기)
        results = self.model.predict(source=cv_img, conf=0.5, device='cpu', verbose=False)
        
        if results and len(results) > 0:
            result = results[0]
            
            # 마스크(세그멘테이션 영역)가 감지되었을 때만 실행
            if result.masks is not None:
                masks = result.masks.xy  # 폴리곤(다각형) 좌표 리스트
                boxes = result.boxes     # 바운딩 박스 및 신뢰도 정보

                for mask, box in zip(masks, boxes):
                    # --- 1) 파란색 마스크 칠하기 ---
                    pts = np.int32([mask])
                    overlay = annotated_img.copy()
                    
                    # BGR 포맷이므로 파란색은 (255, 0, 0)
                    cv2.fillPoly(overlay, pts, color=(255, 0, 0)) 
                    # 원본 이미지와 50% 비율로 섞어서 반투명하게 만듦
                    cv2.addWeighted(overlay, 0.5, annotated_img, 0.5, 0, annotated_img)
                    
                    # 마스크 테두리도 파란색 실선으로 뚜렷하게 그리기
                    cv2.polylines(annotated_img, pts, isClosed=True, color=(255, 0, 0), thickness=2)

                    # --- 2) 신뢰도 (0.88 형식) 띄우기 ---
                    conf = float(box.conf[0])
                    conf_text = f"{conf:.2f}" # 소수점 2자리까지만 포맷팅
                    
                    # 텍스트 띄울 위치 계산 (폴리곤의 맨 위쪽 근처)
                    text_x = int(np.min(pts[0][:, 0]))
                    text_y = int(np.min(pts[0][:, 1])) - 10
                    text_y = max(20, text_y) # 화면 위로 글씨가 짤리지 않게 보정

                    cv2.putText(annotated_img, conf_text, (text_x, text_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

        # 4. 결과 이미지를 다시 ROS 2 토픽으로 발행
        out_msg = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloLaneSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()