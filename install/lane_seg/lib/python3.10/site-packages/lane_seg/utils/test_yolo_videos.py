import cv2
import numpy as np
from ultralytics import YOLO

# 경로 세팅
model_path = '/home/woonggook/Downloads/second_best.pt'
video_path = '/home/woonggook/Desktop/test_videos/track_record_20260225_163600.avi'

# 모델 로드
model = YOLO(model_path)

# 비디오 열기
cap = cv2.VideoCapture(video_path)

print("🚀 실시간 영상 창을 띄웁니다! (끄려면 영상 창을 선택하고 키보드 'q'를 누르세요)")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("✅ 영상 재생이 끝났습니다.")
        break

    # 프레임의 가로(w), 세로(h) 크기 가져오기
    h, w = frame.shape[:2]

    # 예측 (빠른 처리를 위해 imgsz=320)
    results = model.predict(frame, conf=0.7, imgsz=320, verbose=False)
    result = results[0]

    # 차선(마스크)이 인식되었을 때만 파란색으로 칠하기
    if result.masks is not None:
        masks = result.masks.data.cpu().numpy()
        boxes = result.boxes.data.cpu().numpy()

        for mask, box in zip(masks, boxes):
            # 1. 마스크를 원본 해상도에 맞게 쫙 펴주기
            mask_resized = cv2.resize(mask, (w, h))
            
            # 2. 파란색(B=255, G=0, R=0) 덧칠할 빈 도화지 생성
            color_mask = np.zeros_like(frame)
            color_mask[mask_resized > 0.5] = [255, 0, 0]

            # 3. 원본 영상과 합성 (투명도 0.5)
            frame = cv2.addWeighted(frame, 1, color_mask, 0.5, 0)

            # 4. 확률(%) 텍스트 박스 근처에 띄우기
            conf = box[4]
            x1, y1 = int(box[0]), int(box[1])
            text = f"{conf*100:.1f}%"

            # 5. 가독성을 위해 검은 테두리 먼저 그리고 파란 글씨 쓰기
            y_text = max(30, y1 - 15)
            cv2.putText(frame, text, (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(frame, text, (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    # 🎬 화면에 바로 띄우기! (창 이름: Lane Detection Live)
    cv2.imshow('Lane Detection Live', frame)

    # 'q' 키를 누르면 즉시 종료 (1ms 대기)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("🛑 사용자가 영상을 종료했습니다.")
        break

# 작업 끝나면 창 닫고 메모리 해제
cap.release()
cv2.destroyAllWindows()