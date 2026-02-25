import cv2
import os

# --- 📂 바탕화면 경로 자동 설정 ---
home_dir = os.path.expanduser('~')

# 1. 설정: 동영상 파일 경로와 저장할 폴더 이름
VIDEO_NAME = 'track_record.avi'  # 돌리실 영상 이름으로 계속 바꿔주시면 됩니다!
VIDEO_PATH = os.path.join(home_dir, 'Desktop', 'test_videos', VIDEO_NAME)

OUTPUT_DIR = os.path.join(home_dir, 'Desktop', 'real_datasets')
os.makedirs(OUTPUT_DIR, exist_ok=True)
# --------------------------------

# 영상 이름에서 확장자(.avi)만 쏙 빼서 진짜 이름만 가져오기 (예: 'track_record')
video_base_name = os.path.splitext(VIDEO_NAME)[0]

cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    print(f"❌ 동영상 파일을 찾을 수 없습니다!\n경로를 확인해주세요: {VIDEO_PATH}")
    exit()

fps = cap.get(cv2.CAP_PROP_FPS)
print(f"🎥 원본 영상 FPS: {fps}")

interval = int(fps * 0.2)
if interval == 0:
    interval = 1

print(f"⏱️ 0.2초 간격({interval} 프레임당 1장)으로 추출을 시작합니다...")

frame_count = 0
saved_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    if frame_count % interval == 0:
        # 🔥 이 부분이 핵심입니다! 사진 이름에 영상 이름을 붙여서 덮어쓰기 원천 차단!
        file_name = f"{video_base_name}_{saved_count:04d}.jpg"
        save_path = os.path.join(OUTPUT_DIR, file_name)
        
        cv2.imwrite(save_path, frame)
        saved_count += 1
        
    frame_count += 1

cap.release()
print(f"\n✅ 추출 완료! 총 {saved_count}장의 이미지가 바탕화면의 'real_datasets' 폴더에 저장되었습니다.")