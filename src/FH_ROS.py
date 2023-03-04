#!/usr/bin/env python

from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import rospy
import cv2
import numpy as np
# import Funcs as gf
import os
# import tensorrt as trt
import time
import sys
from cv_bridge import CvBridge

import itertools

import torch
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn
from torchsummary import summary as summary_
from torch.nn import functional as F
import torchvision
from torchvision import datasets, transforms

# from albumentation import Resize, Compose

use_cuda = torch.cuda.is_available()

# from grid_map_msgs.msg import GridMap

# from torch2trt import TRTModule

# #print(os.getcwd())
# model=load_model('LegoNet_V7_FP16')
# #model.predict(np.array(initpred).reshape(-1, 16, 16, 1))
# saved_model_loaded = tf.saved_model.load('LegoNet_V7_FP16', tags=[tag_constants.SERVING])
# result = saved_model_loaded.signatures['serving_default']
# #initpred = [0]*256
# #result = result(np.array(initpred).reshape(16, 16))
# signature_keys = list(saved_model_loaded.signatures.keys())
# print(signature_keys)
use_cuda = torch.cuda.is_available()

img_count = 0

class CNNClassifier(nn.Module):

    def __init__(self):
        # 항상 torch.nn.Module을 상속받고 시작
        super(CNNClassifier, self).__init__()
        self.conv1 = nn.Conv2d(1, 8, 9, stride=1, padding=4)  # 6@24*24

        self.conv2 = nn.Conv2d(8, 8, 5, stride=1, padding=2)  # 16@8*8

        # self.conv3 = nn.Conv2d(8, 8, 3, stride=1, padding=1) # 16@8*8

        self.flatten = nn.Flatten()

        self.fc1 = nn.Linear(128, 50, bias=True)

        self.fc2 = nn.Linear(50, 256, bias=True)

        # self.conv_module = nn.Sequential(
        #     self.conv1,
        #     self.conv2,
        #     self.fc1,
        #     self.fc2
        # )
        # # self.fc_module = nn.Sequential(

        if use_cuda:
            self.conv1 = self.conv1.cuda()
            self.conv2 = self.conv2.cuda()
            self.flatten = self.flatten.cuda()
            # self.conv3 = self.conv3.cuda()
            self.fc1 = self.fc1.cuda()
            self.fc2 = self.fc2.cuda()

    def forward(self, x):

        x = F.max_pool2d(F.relu(self.conv1(x)), 2)
        x = F.max_pool2d(F.relu(self.conv2(x)), 2)
        # x = F.relu(self.conv2(x))
        # x = F.max_pool2d(F.relu(self.conv3(x)), 2)
        # x = x.view(-1, 8 * 4 * 4)
        # print(x.shape)
        x = self.flatten(x)

        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        # print(x.shape)
        # return F.softmax(x, dim=1)
        return x


cnn = CNNClassifier()
# cnn.load_state_dict(torch.load('/home/kch/Desktop/ver2epoch1000.pth'))
# cnn.eval()

# cnn = TRTModule()

cnn.load_state_dict(torch.load('/home/xavier/Desktop/ver3epoch1000normal.pth'))

label_pub = None

# def test_one_image(I):
#     '''
#     I - 28x28 uint8 numpy array
#     '''
#     # test phase
#     cnn.eval()
#     # convert image to torch tensor and add batch dim
#     batch = torch.tensor(I / 255).unsqueeze(0)
#     # We don't need gradients for test, so wrap in
#     # no_grad to save memory
#     with torch.no_grad():
#         batch = batch.to(device)
#         # forward propagation
#         output = model( batch )
#         # get prediction
#         output = torch.argmax(output, 1)
#      return output


def predict_callback(img):
    submap = Image()
    submap.data = list(img.data)
    label = Int16()
    # print(submap.data)

    # input_image.append(submap)
    try:
        submap.data = np.array(submap.data).reshape(16, 16)
        submap.data = submap.data.astype('float32')

        img = np.zeros((16, 16, 3)).astype(np.uint8)
        img[:, :, 0], img[:, :, 1], img[:, :,2] = submap.data, submap.data, submap.data

        # submap = submap.astype('float32')
        # submap.data = gf.nullfix(submap.data)
        # print(submap.data)
        submap.data = ((submap.data/255.0)-0.5)/0.5
        # submap.data = cv2.normalize(submap.data,None,-1.0,1.0,norm_type=cv2.NORM_MINMAX,dtype=cv2.CV_32F)

        # submap.data = gf.shaping(submap.data)

        submap.data = np.array(submap.data).reshape(1, 1, 16, 16)
        b = torch.Tensor(submap.data)

        # print(b)
        # b = preprocess(submap.data)
        # b = preprocess1(submap.data)
        # b = preprocess2(b)
        # print(b)
        b = b.cuda()
        # x = preprocess(submap)
        # print(x)
        # print(submap.data)
        # image = img
        # x = image
        # x = image.img_to_array(x)
        # x = np.expand_dims(submap.data, axis=0)
        # x = preprocess_input(x)
        # x = tf.constant(submap.data, dtype=tf.float32)
        # submap.data = tf.constant(submap.data, dtype=tf.float32)
        # print(result.structured_outputs)

        # predic = np.argmax(result, axis=0)
        t1 = time.time()
        trueresult = cnn(b)
        predic = torch.argmax(trueresult)
        predic = int(predic)
        t2 = time.time()

        img_x, img_y = int(predic/16), int(predic % 16)
        
        # for i in range(3):
        # if abs(img[img_x, img_y, 0] - img[min(15, img_x +1), img_y, 0]) > 10:
        #     img_x = max(0, img_x - 1)
        #     img_x = max(0, img_x - 1)

        # if abs(img[img_x, img_y, 0] - img[max(0, img_x - 1), img_y, 0]) > 10:
        #     img_x = min(15, img_x + 1)
        #     img_x = min(15, img_x + 1)
        o_x, o_y = img_x, img_y
        for i in range(2): 
            if abs(img[img_x, img_y, 0] - img[img_x, min(15, img_y + 2), 0]) > 10:
                img_y = max(0, img_y - 1)

            if abs(img[img_x, img_y, 0] - img[img_x, min(15, img_y + 1), 0]) > 10:
                img_y = max(0, img_y - 1)
                img_y = max(0, img_y - 1)

            if abs(img[img_x, img_y, 0] - img[img_x, max(0, img_y - 1), 0]) > 10:
                img_y = min(15, img_y + 1)
                img_y = min(15, img_y + 1)
        # img[o_x, o_y, 0], img[o_x, o_y, 1], img[o_x, o_y, 2] = 255, 0, 0 # original;
        img[img_x, img_y, 0], img[img_x, img_y, 1], img[img_x, img_y, 2] = 0, 0, 255

        img_publisher(submap_pub, img)

        predic = 16*img_y + img_x
        # print(t2-t1)
    except:
        predic = 120
    #predic = int(predic)
    label.data = predic
    label_pub.publish(predic)
    # global img_count
    # np.save("/home/xavier/Documents/img_"+str(img_count),img)
    # img_count += 1
    # print("img_count",img_count)
    # print(predic)
    # print(trueresult)
    


def predict_callback2(img):
    submap = Image()
    submap.data = list(img.data)
    label = Int16()
    # print(submap.data)

    # input_image.append(submap)
    try:
        submap.data = np.array(submap.data).reshape(16, 16)
        submap.data = submap.data.astype('float32')
        # submap = submap.astype('float32')

        img = np.zeros((16, 16, 3)).astype(np.uint8)
        img[:, :, 0], img[:, :, 1], img[:, :,2] = submap.data, submap.data, submap.data

        submap.data = ((submap.data/255.0)-0.5)/0.5
        # submap.data = cv2.normalize(submap.data,None,-1.0,1.0,norm_type=cv2.NORM_MINMAX,dtype=cv2.CV_32F)
        # submap.data = gf.nullfix(submap.data)
        # submap.data = gf.shaping(submap.data)

        submap.data = np.array(submap.data).reshape(1, 1, 16, 16)
        b = torch.Tensor(submap.data)

        # print(b)
        # b = preprocess(submap.data)
        # b = preprocess1(submap.data)
        # b = preprocess2(b)
        # print(b)
        b = b.cuda()
        # x = preprocess(submap)
        # print(x)
        # print(submap.data)
        # image = img
        # x = image
        # x = image.img_to_array(x)
        # x = np.expand_dims(submap.data, axis=0)
        # x = preprocess_input(x)
        # x = tf.constant(submap.data, dtype=tf.float32)
            # submap.data = tf.constant(submap.data, dtype=tf.float32)
        # print(result.structured_outputs)

        # predic = np.argmax(result, axis=0)
        t3 = time.time()
        trueresult = cnn(b)
        predic = torch.argmax(trueresult)
        predic = int(predic)
        t4 = time.time()

        img_x, img_y = int(predic/16), int(predic % 16)
        # if abs(img[img_x, img_y, 0] - img[min(15, img_x +1), img_y, 0]) > 10:
        #     img_x = max(0, img_x - 1)
        #     img_x = max(0, img_x - 1)

        # if abs(img[img_x, img_y, 0] - img[max(0, img_x - 1), img_y, 0]) > 10:
        #     img_x = min(15, img_x + 1)
        #     img_x = min(15, img_x + 1)
        for i in range(2):
            
            if abs(img[img_x, img_y, 0] - img[img_x, min(15, img_y + 1), 0]) > 10:
                img_y = max(0, img_y - 1)
                img_y = max(0, img_y - 1)
            
            
            if abs(img[img_x, img_y, 0] - img[img_x, max(0, img_y - 1), 0]) > 10:
                img_y = min(15, img_y + 1)
                img_y = min(15, img_y + 1)

        img[img_x, img_y, 0], img[img_x, img_y, 1], img[img_x, img_y, 2] = 0, 0, 255

        predic = 16*img_y + img_x
        # print(t4-t3)
    
    except:
        predic = 120
    #predic = int(predic)
    label.data = predic
    label2_pub.publish(predic)
    # print(predic)
    # print(trueresult)
    


def img_publisher(pub, data):
    br = CvBridge()
    msg = br.cv2_to_imgmsg(data)
    if data is not None:
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('CNN')
    rospy.Subscriber('/submap_image', Image, predict_callback)
    rospy.Subscriber('/submap2_image', Image, predict_callback2)
    label_pub = rospy.Publisher('CNN_Foothold', Int16, queue_size=10)
    label2_pub = rospy.Publisher('CNN_Foothold2', Int16, queue_size=10)
    submap_pub = rospy.Publisher('CNN_submap', Image, queue_size=10)
    #sys.stdout = open('output.txt','w')
    rospy.spin()

# rosbag record -O subset /elevation_mapping/elevation_map_raw /submap /submap2 /simple_class_node/marker1 /simple_class_node/marker1_before /simple_class_node/marker2 /simple_class_node/marker2_before /CNN_submap /output

