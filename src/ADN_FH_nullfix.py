from __future__ import absolute_import, division, print_function, unicode_literals

import cv2
import tensorflow as tf

import os #디렉토리 경로 호출할때 사용
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import natsort

from tensorflow import keras
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.callbacks import ModelCheckpoint
# from sklearn.preprocessing import LabelEncoder
# from sklearn.preprocessing import OneHotEncoder
# from sklearn.preprocessing import StandardScaler
# from sklearn.model_selection import KFold
# from sklearn.model_selection import train_test_split
from tensorflow.keras.utils.np_utils import to_categorical


def nullfix(img):
    for k in range(img.shape[0]):
        for i in range(img.shape[1]):
            if img[k,i] == 0:
                img[k,i]=84
    return img



TRAIN_DIR = '/home/kch/Desktop/datast1trva/train' #이미지 데이터 주소 ex) 'MNIST/trainingSet/'
train_folder_list = np.array(os.listdir(TRAIN_DIR)) #training dataset 내부에 존재하는 폴더명을 어레이로 저장
#train_folder_list = ['folder1' 'folder2' 'forder3'...]
# NPY_DIR = '/media/kkw/Samsung_T7/CNN/ADN_FH_DATA'
train_folder_list = natsort.natsorted(train_folder_list)
integer_encoded = []

train_input = []
train_label = []

for i in range(len(train_folder_list)):
    integer_encoded.append(int(train_folder_list[i]))
#train_folder_list를 숫자형 array로 반환 : integer_encoded = [0 1 2...]


for index in range(len(train_folder_list)):
    path = os.path.join(TRAIN_DIR, train_folder_list[index])
    path = path + '/'
    #예시. path = 'C:/User/YJ/Desktop/과제/MNIST/trainingSet/0_zero/'
    #그냥 os.path.join 마지막에 , 하나 추가해주면 그 밑에 줄은 필요 없는거 아닌가?
    img_list = os.listdir(path) # 해당 폴더내의 파일명을 배열로 저장

    for img in img_list:
        img_path = os.path.join(path, img) #정확한 이미지 경로 저장
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        img = nullfix(img)
        train_input.append([np.array(img)])
        train_label.append(integer_encoded[index]) # label 번호 순서대로 저장해줌


train_input = np.array(train_input).astype(np.float32)
train_label = np.array(train_label)


train_label = to_categorical(train_label, num_classes=256)

train_input = train_input.reshape(-1, 16, 16)/255



x_input=train_input.reshape(train_input.shape[0], train_input.shape[1], train_input.shape[2], 1)


#Validation Data 나눠주기
x_train, x_val, h_train, h_val = train_test_split(x_input, train_label, test_size=0.2, random_state = 72)

#reshape 해주기 전 상태가 (32, 1, 15, 15, 3)상태여서 reshape 필요

np.save('/home/kch/Desktop/NPY_Data/x_input_npy', x_input)
np.save('/home/kch/Desktop/NPY_Data/h_label_npy', train_label)




#Test data 만들기
TEST_DIR = '/home/kch/Desktop/dataset_test_V2'
test_folder_list = np.array(os.listdir(TEST_DIR))
test_folder_list = natsort.natsorted(test_folder_list)

test_input = []
test_label = []

for index in range(len(test_folder_list)):
    path = os.path.join(TEST_DIR, test_folder_list[index])
    path = path + '/'
    img_list = os.listdir(path)
    for img in img_list:
        img_path = os.path.join(path, img)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        img = nullfix(img)
        test_input.append([np.array(img)])
        test_label.append(integer_encoded[index])

test_input = np.array(test_input).astype(np.float32)
test_label = np.array(test_label)


test_label = to_categorical(test_label, num_classes=256)

test_input = test_input.reshape(-1, 16, 16)/255

test_input = test_input.reshape(test_input.shape[0], test_input.shape[1], test_input.shape[2], 1)
np.save('/media/kch/Samsung_T7/CNN/ADN_FH_DATA/test_input.npy', test_input)
np.save('/media/kch/Samsung_T7/CNN/ADN_FH_DATA/test_label.npy', test_label)




#CNN 모델 생성
model = tf.keras.Sequential([
    tf.keras.layers.Conv2D(8, (11,11), activation='relu', padding='same', strides=1, input_shape=(16,16,1)),
    tf.keras.layers.MaxPool2D((2,2), strides=2),
    tf.keras.layers.Conv2D(8, (5,5), activation='relu', padding='same', strides=1),
    tf.keras.layers.MaxPool2D((2,2), strides=2),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(20, activation='relu'),
    tf.keras.layers.Dense(256,activation='softmax')
])

model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['acc'])

es = EarlyStopping(monitor='val_loss', patience=20, mode='min')
mc = ModelCheckpoint('Model/best_model.h5', monitor='val_loss', mode='min', save_best_only=True)

# results = model.fit(x_train, h_train, epochs=500, validation_data=(x_val, h_val), callbacks=[es, mc]) #(height map input, output, epochs(반복수)=, validation_data=(test input, test output))
results = model.fit(x_train, h_train, epochs=5, validation_data=(x_val, h_val))

model.save('imsi.h5')

score = model.evaluate(test_input, test_label, verbose=1)
print('test_acc=',score[1],'test_loss=',score[0])

#test_loss, test_acc = model.evaluate(test_input, test_label, verbose=1)

print(results.history.keys())
#summarize history for accuracy
plt.plot(results.history['acc'])
plt.plot(results.history['val_acc'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()
#summarize history for loss
plt.plot(results.history['loss'])
plt.plot(results.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()
