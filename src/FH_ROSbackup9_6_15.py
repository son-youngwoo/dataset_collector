#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import Funcs as gf
import os
import tensorflow as tf
import tensorrt as trt
import time
import sys

from tensorflow import keras
from tensorflow.keras.models import load_model
from tensorflow.keras.applications.resnet50 import preprocess_input
from tensorflow.keras.preprocessing import image

from tensorflow.python.saved_model import tag_constants
from tensorflow.keras.applications.resnet50 import ResNet50
from tensorflow.keras.applications.resnet50 import preprocess_input, decode_predictions
# from grid_map_msgs.msg import GridMap
from std_msgs.msg import Int16
from sensor_msgs.msg import Image

print(os.getcwd())
model=load_model('LegoNet_V7_FP16')
#model.predict(np.array(initpred).reshape(-1, 16, 16, 1))
saved_model_loaded = tf.saved_model.load('LegoNet_V7_FP16', tags=[tag_constants.SERVING])
result = saved_model_loaded.signatures['serving_default']
initpred = [0]*256
result = result(np.array(initpred).reshape(16, 16))
signature_keys = list(saved_model_loaded.signatures.keys())
print(signature_keys)


label_pub = None

def predict_callback(img):
    submap = Image()
    submap.data = list(img.data)
    label = Int16()
    # print(submap.data)



    submap.data = np.array(submap.data).reshape(16, 16)
    #submap.data = gf.nullfix(submap.data)
    #submap.data = gf.shaping(submap.data)
    print(submap.data)
   # image = img
   # x = image
   # x = image.img_to_array(x)
   # x = np.expand_dims(submap.data, axis=0)
   # x = preprocess_input(x)
   # x = tf.constant(submap.data, dtype=tf.float32) 

   # print(result.structured_outputs)
   
  #  predic = np.argmax(result, axis=0)
    t1=time.time()
    trueresult = result(submap.data)
    predic = np.argmax(trueresult, axis=0)
    t2=time.time()
    predic = int(predic)
    label.data = predic
    label_pub.publish(label)
    #print(label)
    print(t2-t1)

if __name__ == '__main__':
    rospy.init_node('CNN')
    rospy.Subscriber('/submap_image', Image, predict_callback)
    label_pub = rospy.Publisher('CNN_Foothold', Int16, queue_size=10)
    sys.stdout = open('output.txt','w')
    rospy.spin()
