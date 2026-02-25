import cv2
import numpy as np
import os
import random

# ==========================================
# 1. 기본 설정 (해상도 및 규격)
# ==========================================
PX_PER_CM = 1
IMG_W, IMG_H = 240, 300
CENTER_X = IMG_W // 2

LANE_WIDTH_TOTAL = 20   # 검은바탕 총 20cm
LANE_WIDTH_WHITE = 10   # 가운데 흰색 10cm
TRACK_WIDTH_HALF = 75   # 주행폭 절반 (75cm)

# 바탕화면에 확실하게 폴더 생성
home_dir = os.path.expanduser('~')
SAVE_DIR = os.path.join(home_dir, 'Desktop', 'synthetic_dataset')
IMG_DIR = os.path.join(SAVE_DIR, "images")
MASK_DIR = os.path.join(SAVE_DIR, "masks")
os.makedirs(IMG_DIR, exist_ok=True)
os.makedirs(MASK_DIR, exist_ok=True)

# ==========================================
# 2. 배경 생성 함수 (이전과 동일)
# ==========================================
def create_asphalt_bg(w, h):
    bg = np.full((h, w, 3), (100, 100, 100), dtype=np.uint8)
    noise = np.random.randint(0, 50, (h, w, 3), dtype=np.uint8)
    return cv2.add(bg, noise)

def create_grass_bg(w, h):
    bg = np.full((h, w, 3), (34, 139, 34), dtype=np.uint8)
    noise = np.random.randint(0, 60, (h, w, 3), dtype=np.uint8)
    bg = cv2.add(bg, noise)
    blur = cv2.GaussianBlur(bg, (5, 5), 0)
    return cv2.addWeighted(bg, 0.7, blur, 0.3, 0)

def create_sidewalk_bg(w, h):
    bg = np.full((h, w, 3), (180, 180, 180), dtype=np.uint8)
    for y in range(0, h, 40): cv2.line(bg, (0, y), (w, y), (150, 150, 150), 2)
    for x in range(0, w, 40): cv2.line(bg, (x, 0), (x, h), (150, 150, 150), 2)
    noise = np.random.randint(0, 30, (h, w, 3), dtype=np.uint8)
    return cv2.subtract(bg, noise)

# ==========================================
# 3. 핵심: 2차 함수 곡선으로 차선(폴리곤) 그리기
# ==========================================
def get_curve_polygon(x_centers, y_pts, width):
    """중심 좌표(x_centers)를 따라 일정한 두께(width)를 가진 다각형(Polygon)을 만듭니다."""
    half_w = width / 2.0
    left_pts = np.vstack((x_centers - half_w, y_pts)).T
    right_pts = np.vstack((x_centers + half_w, y_pts)).T
    # 다각형을 닫기 위해 오른쪽 점들은 역순으로 붙임
    poly_pts = np.vstack((left_pts, right_pts[::-1]))
    return np.int32([poly_pts])

def draw_curved_lanes_and_mask(bg_img, shift_x, curve_a, curve_b):
    h, w, c = bg_img.shape
    image_canvas = bg_img.copy()
    mask_canvas = np.zeros((h, w), dtype=np.uint8)

    # 화면 맨 아래(y=h)에서 위(y=0)로 올라가면서 곡선 좌표 계산
    y_pts = np.linspace(0, h, 50)
    y_from_bottom = h - y_pts  # 차가 있는 맨 아래가 기준점(0)

    # 2차 함수 곡선 방정식: x = A*y^2 + B*y + C
    # 중앙 궤적 계산 (C는 shift_x로 치우침 표현)
    x_track = (w // 2) + shift_x + (curve_a * (y_from_bottom ** 2)) + (curve_b * y_from_bottom)

    # 좌/우 차선의 정중앙 x좌표 계산 (1.5m 폭 유지)
    x_left_lane = x_track - TRACK_WIDTH_HALF
    x_right_lane = x_track + TRACK_WIDTH_HALF

    # 폴리곤(다각형) 형태 생성
    left_black_poly = get_curve_polygon(x_left_lane, y_pts, LANE_WIDTH_TOTAL)
    left_white_poly = get_curve_polygon(x_left_lane, y_pts, LANE_WIDTH_WHITE)
    
    right_black_poly = get_curve_polygon(x_right_lane, y_pts, LANE_WIDTH_TOTAL)
    right_white_poly = get_curve_polygon(x_right_lane, y_pts, LANE_WIDTH_WHITE)

    # --- 이미지에 그리기 ---
    # 1. 두꺼운 검은색 선 먼저 바닥에 깔기 (좌/우)
    cv2.fillPoly(image_canvas, left_black_poly, (20, 20, 20))
    cv2.fillPoly(image_canvas, right_black_poly, (20, 20, 20))

    # 2. 그 정중앙 위에 얇은 흰색 선 덮어 그리기 (좌/우)
    cv2.fillPoly(image_canvas, left_white_poly, (240, 240, 240))
    cv2.fillPoly(image_canvas, right_white_poly, (240, 240, 240))

    # --- 마스크(정답지)에 그리기 ---
    # YOLO 학습 타겟인 가운데 흰색 영역만 255(흰색)로 칠함 (좌/우 단일 클래스)
    cv2.fillPoly(mask_canvas, left_white_poly, 255)
    cv2.fillPoly(mask_canvas, right_white_poly, 255)

    return image_canvas, mask_canvas

# ==========================================
# 4. 메인 실행부
# ==========================================
NUM_SAMPLES = 100 # 시범 50장 (확인 후 500장으로 늘리세요)

print(f"BEV 곡선/직선 통합 데이터 생성을 시작합니다... (총 {NUM_SAMPLES}장)")

for i in range(NUM_SAMPLES):
    bg_type = random.choice(['asphalt', 'grass', 'sidewalk'])
    if bg_type == 'asphalt': bg = create_asphalt_bg(IMG_W, IMG_H)
    elif bg_type == 'grass': bg = create_grass_bg(IMG_W, IMG_H)
    else: bg = create_sidewalk_bg(IMG_W, IMG_H)

    # 랜덤 주행 상황 연출
    shift_x = random.uniform(-60, 60)       # 좌우 쏠림
    
    # 50% 확률로 직선, 50% 확률로 코너 생성
    if random.random() > 0.5:
        curve_a = random.uniform(-0.002, 0.002) # 곡률 (크면 급커브)
        curve_b = random.uniform(-0.3, 0.3)     # 진입 각도 (틀어짐)
    else:
        curve_a = 0                             # 직선
        curve_b = random.uniform(-0.2, 0.2)     # 살짝 삐뚤어짐만 적용

    img, mask = draw_curved_lanes_and_mask(bg, shift_x, curve_a, curve_b)

    img_path = os.path.join(IMG_DIR, f"bev_synthetic_{i:04d}.jpg")
    mask_path = os.path.join(MASK_DIR, f"bev_synthetic_{i:04d}.png")
    
    cv2.imwrite(img_path, img)
    cv2.imwrite(mask_path, mask)

print(f"\n✅ 생성 완료! 바탕화면의 '{SAVE_DIR}' 폴더를 확인하세요.")