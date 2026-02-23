import cv2
import numpy as np  

# 이미지의 전체 경로
image_path = '/home/woonggook/Downloads/image5.png' 
# ----------------------------------------

img = cv2.imread(image_path)
if img is None:
    print(f"[{image_path}] 사진을 찾을 수 없습니다. 경로를 확인해주세요.")
    exit()

height, width, _ = img.shape
print(f"현재 사진 해상도: {width} x {height}")

# 원본 이미지를 보존하기 위해 복사본 생성
display_img = img.copy()
points = []

def get_mouse_points(event, x, y, flags, param):
    global display_img
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append([x, y])
        print(f"Point {len(points)} 추가: ({x}, {y})")
        
        # 클릭한 자리에 빨간 점 표시
        cv2.circle(display_img, (x, y), 5, (0, 0, 255), -1)
        
        # 4개가 다 찍히면 초록색 사다리꼴 선을 그림
        if len(points) == 4:
            pts_array = np.array(points, np.int32)
            cv2.polylines(display_img, [pts_array], True, (0, 255, 0), 2)
            print("\n" + "="*30)
            print("복사해서 bev_node.py에 붙여넣으세요:")
            print(f"self.src_pts = np.float32({points})")
            print("="*30 + "\n")
            print("화면을 닫으려면 아무 키나 누르세요.")
        
        cv2.imshow("Coordinate Picker", display_img)

print("이미지의 4곳을 클릭하세요: [왼쪽위 -> 오른쪽위 -> 오른쪽아래 -> 왼쪽아래]")
cv2.imshow("Coordinate Picker", display_img)
cv2.setMouseCallback("Coordinate Picker", get_mouse_points)

cv2.waitKey(0)
cv2.destroyAllWindows()