import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class BEVCalibrator(Node):
    def __init__(self):
        super().__init__('bev_calibrator')
        
        # ⚠️ 1. 구독할 카메라 토픽 이름을 확인해서 맞춰주세요!
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',  # <- 여기를 사용하시는 토픽명으로 수정!
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        # ⚠️ 2. 체커보드 정보 입력 (반드시 확인!)
        self.checkerboard_size = (9, 8)  # 10x9 칸 → 내부 코너 수 (가로-1, 세로-1)
        self.square_size = 0.08         # 네모 한 칸의 실제 크기 (단위: 미터, 예: 25mm = 0.025)
        
        # 찾아오신 완벽한 내부 파라미터 적용
        self.K = np.array([[484.45878,   0.     , 291.5055 ],
                           [  0.     , 482.5031 , 222.66382],
                           [  0.     ,   0.     ,   1.     ]])
        
        self.D = np.array([0.162875, -0.287212, 0.000692, -0.000678, 0.000000])

        self.get_logger().info("🚀 BEV 캘리브레이터 실행! 영상 창에서 'c'를 누르면 캡처합니다.")

    def image_callback(self, msg):
        # ROS 이미지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 1. 렌즈 왜곡 쫙 펴기!
        undistorted_frame = cv2.undistort(frame, self.K, self.D)
        
        # 화면에 실시간 표시
        cv2.imshow("Live: Undistorted (Press 'c' to Calibrate, 'q' to Quit)", undistorted_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        # 'c' 키를 누르면 캘리브레이션 시작!
        if key == ord('c'):
            self.get_logger().info("📸 캡처 완료! 모서리 탐색 시작...")
            self.calibrate_bev(undistorted_frame)
            
        elif key == ord('q'):
            rclpy.shutdown()

    def calibrate_bev(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 체커보드 모서리 찾기 (플래그로 인식률 향상)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, flags)
        
        if ret:
            # 서브픽셀 단위로 정밀하게 깎기
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # 시각화해서 화면에 1초 띄워줌
            cv2.drawChessboardCorners(frame, self.checkerboard_size, corners2, ret)

            # 4개 모서리 코너에 번호 표시 (순서 확인용)
            debug_frame = frame.copy()
            idx_map = {
                0:                                   ("0:?", (0, 0, 255)),
                self.checkerboard_size[0]-1:         ("1:?", (0, 255, 0)),
                len(corners2)-self.checkerboard_size[0]: ("2:?", (255, 0, 0)),
                len(corners2)-1:                     ("3:?", (0, 255, 255)),
            }
            for idx, (label, color) in idx_map.items():
                pt = tuple(corners2[idx][0].astype(int))
                cv2.circle(debug_frame, pt, 10, color, -1)
                cv2.putText(debug_frame, label, (pt[0]+12, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            cv2.imshow("Corners Found! (0=RED 1=GREEN 2=BLUE 3=CYAN)", debug_frame)
            self.get_logger().info(f"corners2[0]  (RED)  : {corners2[0][0]}")
            self.get_logger().info(f"corners2[{self.checkerboard_size[0]-1}] (GREEN): {corners2[self.checkerboard_size[0]-1][0]}")
            self.get_logger().info(f"corners2[-{self.checkerboard_size[0]}](BLUE) : {corners2[-self.checkerboard_size[0]][0]}")
            self.get_logger().info(f"corners2[-1] (CYAN) : {corners2[-1][0]}")
            cv2.waitKey(3000)

            # --- BEV(탑뷰) 매핑 로직 ---
            # 화면에서 찾은 4개 모서리 좌표 (좌상, 우상, 좌하, 우하)
            # RED->좌상, GREEN->우상, BLUE->좌하, CYAN->우하 순서가 맞는지 확인 후 사용
            src_pts = np.float32([
                corners2[self.checkerboard_size[0]-1][0],    # 좌상 (GREEN)
                corners2[-1][0],                             # 우상 (YELLOW)
                corners2[0][0],                              # 좌하 (RED)
                corners2[-self.checkerboard_size[0]][0],     # 우하 (BLUE)
            ])

            # 실제 바닥 도면(BEV)에서의 4개 모서리 좌표 생성 (1픽셀 = 1cm 로 가정)
            # px_per_meter를 100으로 잡으면 (1m=100px) 직관적입니다.
            px_per_meter = 100 
            w = (self.checkerboard_size[0] - 1) * self.square_size * px_per_meter
            h = (self.checkerboard_size[1] - 1) * self.square_size * px_per_meter
            
            # 탑뷰 도화지 중앙쯤에 바둑판이 오도록 여백 세팅
            offset_x = 200
            offset_y = 300
            
            dst_pts = np.float32([
                [offset_x, offset_y],
                [offset_x + w, offset_y],
                [offset_x, offset_y + h],
                [offset_x + w, offset_y + h]
            ])

            # 투시 변환 행렬(Homography) 계산!
            H = cv2.getPerspectiveTransform(src_pts, dst_pts)
            
            # 결과 저장
            save_path = os.path.expanduser('~/ev_ws/src/lane_seg/config/bev_matrix.npy')
            np.save(save_path, H)
            
            self.get_logger().info(f"✅ 축하합니다! 완벽한 BEV 행렬이 저장되었습니다: {save_path}")
            self.get_logger().info("프로그램을 종료합니다.")
            rclpy.shutdown()
        else:
            self.get_logger().error("❌ 체커보드를 찾지 못했습니다! 카메라 앵글이나 조명을 확인하고 다시 'c'를 눌러주세요.")

def main(args=None):
    rclpy.init(args=args)
    calibrator = BEVCalibrator()
    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    calibrator.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()