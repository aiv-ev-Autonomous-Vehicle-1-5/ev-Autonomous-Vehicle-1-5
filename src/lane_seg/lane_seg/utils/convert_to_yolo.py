import cv2, os, glob, shutil, random

# 경로 설정
home = os.path.expanduser('~')
BASE_DIR = os.path.join(home, 'Desktop', 'synthetic_dataset')
IMG_DIR = os.path.join(BASE_DIR, 'images')
MASK_DIR = os.path.join(BASE_DIR, 'masks')
YOLO_DIR = os.path.join(home, 'Desktop', 'yolo_dataset') # 결과물이 저장될 곳

# 폴더 구조 생성
for s in ['train', 'val']:
    os.makedirs(os.path.join(YOLO_DIR, 'images', s), exist_ok=True)
    os.makedirs(os.path.join(YOLO_DIR, 'labels', s), exist_ok=True)

# 변환 로직
img_files = glob.glob(os.path.join(IMG_DIR, '*.jpg'))
random.shuffle(img_files)
train_idx = int(len(img_files) * 0.8)

for i, img_path in enumerate(img_files):
    name = os.path.splitext(os.path.basename(img_path))[0]
    mask_path = os.path.join(MASK_DIR, name + '.png')
    split = 'train' if i < train_idx else 'val'
    
    # 이미지 복사
    shutil.copy(img_path, os.path.join(YOLO_DIR, 'images', split, name + '.jpg'))
    
    # 마스크를 텍스트 좌표로 변환
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    with open(os.path.join(YOLO_DIR, 'labels', split, name + '.txt'), 'w') as f:
        for cnt in contours:
            if cv2.contourArea(cnt) < 50: continue
            line = "0 "
            for pt in cnt:
                line += f"{pt[0][0]/240:.6f} {pt[0][1]/300:.6f} "
            f.write(line.strip() + "\n")

# data.yaml 자동 생성
yaml_content = f"path: {YOLO_DIR}\ntrain: images/train\nval: images/val\nnames:\n  0: lane"
with open(os.path.join(YOLO_DIR, 'data.yaml'), 'w') as f:
    f.write(yaml_content)

print(f"✅ 변환 완료! 바탕화면에 'yolo_dataset' 폴더가 생겼습니다.")