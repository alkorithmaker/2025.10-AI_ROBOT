"""
Author : Byunghyun Ban
Date : 2020.07.24.
"""
import numpy as np
from tensorflow import keras
import data_reader

# 몇 에포크 만큼 학습을 시킬 것인지 결정합니다.
EPOCHS = 50  # 예제 기본값은 50입니다.

# 데이터를 읽어옵니다.
dr = data_reader.DataReader()

# 인공신경망을 제작합니다.
model = keras.Sequential([
    keras.layers.Dense(6),
    keras.layers.Dense(256, activation="relu"),
    keras.layers.Dense(256, activation="relu"),
    keras.layers.Dense(256, activation="relu"),
    keras.layers.Dense(256, activation="relu"),
    keras.layers.Dense(2, activation='sigmoid')
])

# 인공신경망을 컴파일합니다.
model.compile(optimizer="adam", loss="mse", metrics=['mae'])

# 인공신경망을 학습시킵니다.
print("\n\n************ TRAINING START ************ ")
early_stop = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
history = model.fit(dr.train_X, dr.train_Y, epochs=EPOCHS,
                    validation_data=(dr.test_X, dr.test_Y),
                    callbacks=[early_stop])

print("Height Factor = ", dr.normalize_factors[-2])
print("Weight Factor = ", dr.normalize_factors[-1])

Truth = dr.test_Y
Predict = np.array(model(dr.test_X))
Truth[:,0] *= dr.normalize_factors[-2]
Predict[:,0] *= dr.normalize_factors[-2]
Truth[:,1] *= dr.normalize_factors[-1]
Predict[:,1] *= dr.normalize_factors[-1]

B = Predict[:,1]/(Predict[:,0]/100 * Predict[:,0]/100)
B = B[:,np.newaxis]
Predict = np.hstack((Predict, B))

B = Truth[:,1]/(Truth[:,0]/100*Truth[:,0]/100)
B = B[:, np.newaxis]
Truth = np.hstack((Truth, B))

Result = np.hstack((Truth, Predict))
np.savetxt("Result.csv", Result, fmt='%3.1f', delimiter=',')

data_reader.draw_graph(Predict[:,2], Truth[:,2], history)
# 학습 결과를 그래프로 출력합니다.
data_reader.draw_graph(model(dr.test_X), dr.test_Y, history)
