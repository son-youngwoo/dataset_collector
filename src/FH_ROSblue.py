#!/usr/bin/env python
import rospy
#import cv2
import numpy as np
#import Funcs as gf
import os
# import tensorrt as trt
import time
import sys

import itertools

import torch 
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn
#from torchsummary import summary as summary_
from torch.nn import functional as F
#import torchvision
#from torchvision import datasets, transforms

use_cuda = torch.cuda.is_available()

# from grid_map_msgs.msg import GridMap
from std_msgs.msg import Int16
from sensor_msgs.msg import Image

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

class CNNClassifier(nn.Module):
    
    def __init__(self):
        # 항상 torch.nn.Module을 상속받고 시작
        super(CNNClassifier, self).__init__()
        self.conv1 = nn.Conv2d(1, 8, 11, stride=1, padding=5) # 6@24*24

        self.conv2 = nn.Conv2d(8, 8, 5, stride=1, padding=2) # 16@8*8

        # self.conv3 = nn.Conv2d(8, 8, 3, stride=1, padding=1) # 16@8*8

        self.flatten = nn.Flatten()

        self.fc1 = nn.Linear(128, 20, bias=True)

        self.fc2 = nn.Linear(20, 256, bias=True)

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
        return x
        # return x

cnn = CNNClassifier()
cnn.load_state_dict(torch.load('/home/kch/Desktop/ver2epoch1000.pth'))
cnn.eval()

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
preprocess1 = transforms.ToTensor()
preprocess2 = transforms.Normalize((0.5),(0.5))

def predict_callback(img):
    submap = Image()
    submap.data = list(img.data)
    label = Int16()
    # print(submap.data)


    # input_image.append(submap)


    submap.data = np.array(submap.data).reshape(16, 16)
    submap.data = submap.data.astype('float32')
    # submap = submap.astype('float32')
    submap.data = submap.data/255.0
    # submap.data = cv2.normalize(submap.data,None,-1.0,1.0,norm_type=cv2.NORM_MINMAX,dtype=cv2.CV_32F)
    # submap.data = gf.nullfix(submap.data)
    # submap.data = gf.shaping(submap.data)

    submap.data = np.array(submap.data).reshape(1,1,16,16)
    b=torch.Tensor(submap.data)
    # b = preprocess(submap.data)
    # b = preprocess1(submap.data)
    # b = preprocess2(b)
    # print(b)
    b = b.cuda()
    # x = preprocess(submap)
    # print(x)
    #print(submap.data)
   # image = img
   # x = image
   # x = image.img_to_array(x)
   # x = np.expand_dims(submap.data, axis=0)
   # x = preprocess_input(x)
   # x = tf.constant(submap.data, dtype=tf.float32) 
    # submap.data = tf.constant(submap.data, dtype=tf.float32) 
   # print(result.structured_outputs)
   
  #  predic = np.argmax(result, axis=0)
    t1=time.time()
    trueresult = cnn(b)
    predic = torch.argmax(trueresult)
    predic = int(predic)
    t2=time.time()
    #predic = int(predic)
    label.data = predic
    label_pub.publish(predic)
    print(predic)
    # print(trueresult)
    print(t2-t1)

if __name__ == '__main__':
    rospy.init_node('CNN')
    rospy.Subscriber('/submap_image', Image, predict_callback)
    label_pub = rospy.Publisher('CNN_Foothold', Int16, queue_size=10)
    #sys.stdout = open('output.txt','w')
    rospy.spin()
