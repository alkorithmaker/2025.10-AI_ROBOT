# deep_learning_model.py
#--------------------------------------------
#- 로봇 제어의 "지능"을 담당
#- 128 * 128 사이즈의 이미지를 전처리하여 학습함
#- 학습을 토대로 5개의 도형으로 분류하여 값을 출력함
#---------------------------------------------

import tensorflow as tf
from tensorflow.keras import layers, models

# 변수 설정
IMG_SIZE = 128
NUM_CLASSES = 5   
BATCH_SIZE = 32

# 데이터셋 로드
train_ds = tf.keras.utils.image_dataset_from_directory(
    './dataset',
    validation_split=0.2,
    subset="training",
    seed=123,
    image_size=(IMG_SIZE, IMG_SIZE),
    batch_size=BATCH_SIZE,
    color_mode='grayscale'
)

val_ds = tf.keras.utils.image_dataset_from_directory(
    './dataset',
    validation_split=0.2,
    subset="validation",
    seed=123,
    image_size=(IMG_SIZE, IMG_SIZE),
    batch_size=BATCH_SIZE,
    color_mode='grayscale'
)

# 모델 구성
model = models.Sequential([
    # 128 * 128 인풋
    layers.Input(shape=(IMG_SIZE, IMG_SIZE, 1)),

    # 정규화를 가장 먼저 수행
    layers.Rescaling(1./255),

    # 데이터 증강 (도형의 크기,반전과 회전)
    layers.RandomFlip("horizontal_and_vertical"),
    layers.RandomRotation(0.2),
    layers.RandomZoom(0.1),

    # 컨볼루션 2D를 사용하여 CNN 기법을 이용
    layers.Conv2D(32, (3, 3), activation='relu', padding='same'),
    layers.BatchNormalization(), # 학습 속도와 안정성 향상
    layers.MaxPooling2D(),

    layers.Conv2D(64, (3, 3), activation='relu', padding='same'),
    layers.BatchNormalization(),
    layers.MaxPooling2D(),

    layers.Conv2D(128, (3, 3), activation='relu', padding='same'),
    layers.BatchNormalization(),
    layers.MaxPooling2D(),

    layers.Flatten(),
    layers.Dense(256, activation='relu'),
  # 과적합 방지
    layers.Dropout(0.5), 
  # 출력이 2개 이상이므로 softmax 사용 >> 정답이면 0, 오답이면 오답률에 따라 1이상의 값을줌.
  # sottmax는 crossentropy와 적합함.
    layers.Dense(NUM_CLASSES, activation='softmax')
])

# 컴파일 및 콜백 설정
model.compile(
    optimizer='adam',
    loss='sparse_categorical_crossentropy',
    metrics=['accuracy']
)

# 학습 시작
history = model.fit(
    train_ds,
    validation_data=val_ds,
    epochs=30,
)

print("학습 완료")

# 모델 저장 (나중에 불러다 쓰기 위함)
model.save('shape_model_9.keras')
print("\n 모델이 'shape_model_9.keras'로 저장되었습니다.")
