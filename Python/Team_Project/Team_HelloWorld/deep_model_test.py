# deep_model_test.py
#----------------------------------------------
#- 모델을 로봇에 사용하기전에 모형인식이 잘 되는지 확인하는 코드
#- test폴더에서 랜덤으로 사진을 가져와서 전치리 후 모델을 적용
#- 이미지의 예측값을 기반으로 도형의 모습을 출력함
#- 맵플롯을 이용하여 원본 이미지와 영상처리된 이미지를 시각화함
#---------------------------------------------

import tensorflow as tf
import numpy as np
import cv2
import os
import random
import matplotlib.pyplot as plt

# 모델 불러오기 
model = tf.keras.models.load_model('shape_model_9.keras')

# 사진 폴더 경로
test_folder = './test'
file_list = [f for f in os.listdir(test_folder) if f.endswith(('.png', '.jpg', '.jpeg'))]

if not file_list:
    print(" './test' 폴더에 이미지가 없습니다.")
    exit()

random_file = random.choice(file_list)
file_path = os.path.join(test_folder, random_file)
print(f"🎲 선택된 파일: {random_file}")

# 이미지 전처리 함수 (모델과 로직 일치)
def prepare_image(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    
    # 실시간 영상 처리와 동일하게 테두리 5px 제거
    margin = 5
    if img.shape[0] > margin*2 and img.shape[1] > margin*2:
        img = img[margin:-margin, margin:-margin]
    
    # 리사이즈
    img = cv2.resize(img, (128, 128))

    # 실시간/학습 데이터와 이진화 방향 맞추기
    # 만약 배경이 검정, 도형이 흰색이어야 한다면 THRESH_BINARY
    # 반대로 배경이 흰색, 도형이 검정이라면 THRESH_BINARY_INV
    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # 모델 입력용 시각화 이미지 보관
    visual_img = img.copy()
    
    # (1, 128, 128, 1)로 변환
    img = img.reshape(1, 128, 128, 1).astype('float32')
    return img, visual_img

# 예측 실행
processed_img, visual_img = prepare_image(file_path)
prediction = model.predict(processed_img, verbose=0)

# 5개 클래스 순서 (알파벳 순서 주의)
class_names = ['Circle', 'Rectangle', 'Triangle', 'X', 'xother']
result_index = np.argmax(prediction)
confidence = np.max(prediction) * 100

print("-" * 30)
print(f" 모델의 예측: {class_names[result_index]}")
print(f" 확신도: {confidence:.2f}%")
print("-" * 30)

# 결과 화면에 띄우기 (전처리된 이미지와 원본 비교)
plt.figure(figsize=(10, 5))

# 원본 이미지
plt.subplot(1, 2, 1)
plt.imshow(cv2.imread(file_path), cmap='gray')
plt.title("Original Test Image")
plt.axis('off')

# 모델이 실제로 본 이미지 (128x128, 크롭됨)
plt.subplot(1, 2, 2)
plt.imshow(visual_img, cmap='gray')
plt.title(f"Model View: {class_names[result_index]} ({confidence:.1f}%)")
plt.axis('off')

plt.show()
