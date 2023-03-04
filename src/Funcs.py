import numpy as np
#import tensorflow as tf
import cv2

# from keras.models import load_model

# model = load_model('Model/nullfix/noelst_500/FH_11x8+5x8.h5')

def nullfix(img):
    for k in range(img.shape[0]):
        for i in range(img.shape[1]):
            if img[k,i] == 0:
                img[k,i]=127
		#img[k,i]=84
    return img

def shaping(array):
    array = np.array(array).reshape(-1, 16, 16, 1)
    return array

def chlabel(label):
    col,row = divmod(label,16)
    changed = row*16+col
    return changed

#def predict(img):
    img = np.array(img).reshape(16, 16)
    img = nullfix(img)
    img = shaping(img)
    result = model.predict(img)
    label = np.argmax(result, axis=1)
    return int(label[0])
