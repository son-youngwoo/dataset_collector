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

from tensorflow.keras.models import load_model

# from grid_map_msgs.msg import GridMap
from std_msgs.msg import Int16
from sensor_msgs.msg import Image

print(os.getcwd())
model=load_model('11x11x8+5x5x8.h5')
initpred = [0]*256
model.predict(np.array(initpred).reshape(-1, 16, 16, 1))

label_pub = None

def predict_callback(img):
    submap = Image()
    submap.data = list(img.data)
    label = Int16()
    # print(submap.data)
    submap.data = np.array(submap.data).reshape(16, 16)
    submap.data = gf.nullfix(submap.data)
    submap.data = gf.shaping(submap.data)
    #print(submap.data)
    t1=time.time()
    result = model.predict(submap.data)
    t2=time.time()
    #predic = np.argmax(result, axis=0)
    #predic = int(predic)
    #label.data = predic
    #label_pub.publish(label)
    #print(label)
    print(t2-t1)


if __name__ == '__main__':
    rospy.init_node('CNN')
    rospy.Subscriber('/submap_image', Image, predict_callback)
    label_pub = rospy.Publisher('CNN_Foothold', Int16, queue_size=10)
    sys.stdout = open('output2.txt','w')
    rospy.spin()
