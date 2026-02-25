import cv2
import os

# 1. 설정: 동영상 파일 경로와 저장할 폴더 이름
VIDEO_PATH = 'track_record.avi'  # 녹화하신 동영상 이름으로 변경하세요!
# 영상하나마다 폴더 만들기
OUTPUT_DIR = 'real_dataset1'

# 폴더가 없으면 자동 생성
os.makedirs(OUTPUT_DIR, exist_ok=True)

# 2. 동영상 불러오기
cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    print("❌ 동영상 파일을 찾을 수 없습니다. 경로를 확인해주세요!")
    exit()

# 동영상의 원본 FPS(초당 프레임 수) 확인
fps = cap.get(cv2.CAP_PROP_FPS)
print(f"🎥 원본 영상 FPS: {fps}")

# 0.2초마다 프레임을 뽑기 위한 간격 계산 (예: 30fps 영상이면 6프레임마다 1장)
interval = int(fps * 0.2)
if interval == 0:
    interval = 1

print(f"⏱️ 0.2초 간격({interval} 프레임당 1장)으로 추출을 시작합니다...")

frame_count = 0
saved_count = 0

# 3. 프레임 추출 루프
while True:
    ret, frame = cap.read()
    
    # 영상이 끝나면 루프 종료
    if not ret:
        break
        
    # 계산된 간격(0.2초)마다 이미지 저장
    if frame_count % interval == 0:
        file_name = f"real_lane_{saved_count:04d}.jpg"
        save_path = os.path.join(OUTPUT_DIR, file_name)
        
        cv2.imwrite(save_path, frame)
        saved_count += 1
        
    frame_count += 1

cap.release()
print(f"\n✅ 추출 완료! 총 {saved_count}장의 이미지가 '{OUTPUT_DIR}' 폴더에 저장되었습니다.")