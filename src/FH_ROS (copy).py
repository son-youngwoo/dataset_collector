#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import Funcs as gf
import os
# import tensorrt as trt
import time
import sys

import itertools

import torch 
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn
from torchsummary import summary as summary_
from torch.nn import functional as F
import torchvision
from torchvision import datasets, transforms

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
        conv1 = nn.Conv2d(1, 8, 11, stride=1, padding=5) # 6@24*24
        # activation ReLU
        pool1 = nn.MaxPool2d(2) # 6@12*12
        conv2 = nn.Conv2d(8, 8, 5, stride=1, padding=2) # 16@8*8
        # activation ReLU
        pool2 = nn.MaxPool2d(2) # 16@4*4
        #flat = torch.flatten(128)
        # out1 = nn.Linear(128,20)
        # out2 = nn.Linear(20,256)

        self.conv_module = nn.Sequential(
            conv1,
            nn.ReLU(),
            pool1,
            conv2,
            nn.ReLU(),
            pool2,
            nn.Flatten()
            # out1,
            # nn,ReLU(),
            # out2
        )
        
        # fc1 = nn.Linear(8*4*4, 128)
        fc2 = nn.Linear(128, 20)
        # activation ReLU
        fc3 = nn.Linear(20, 256)
        # activation ReLU
        # fc3 = nn.Linear(84, 10)

        # self.fc_module = nn.Sequential(
        #     nn.Flatten(),
        # )

        self.fc_module = nn.Sequential(
            # fc1,
            # nn.ReLU(),
            fc2,
            nn.ReLU(),
            fc3
        )
        
        # # gpu로 할당
        if use_cuda:
            self.conv_module = self.conv_module.cuda()
            self.fc_module = self.fc_module.cuda()
        
    def forward(self, x):
        out = self.conv_module(x) # @16*4*4
        # # make linear
        # dim = 1
        # for d in out.size()[1:]: #16, 4, 4
        #     dim = dim * d
        # out = out.view(-1, dim)
        # print(out.shape)
        out = self.fc_module(out)
        result = F.softmax(out, dim=1)
        return result
    # def forward(self, x):
    #     print("연산 전", x.size())
    #     x = F.relu(self.conv1(x))
    #     print("conv1 연산 후", x.size())
    #     x = F.relu(self.conv2(x))
    #     print("conv2 연산 후",x.size())
    #     x = x.view(-1, 10 * 12 * 12)
    #     print("차원 감소 후", x.size())
    #     x = F.relu(self.fc1(x))
    #     print("fc1 연산 후", x.size())
    #     x = self.fc2(x)
    #     print("fc2 연산 후", x.size())
    #     return x

cnn = CNNClassifier()
cnn.load_state_dict(torch.load('/home/kch/Desktop/dataset/ver1.pth'))
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
#     return output
preprocess = transforms.Compose([
   transforms.ToTensor(),
   transforms.Normalize((0.5,0.5,0.5),(0.5,0.5,0.5))
])

def predict_callback(img):
    submap = Image()
    submap.data = list(img.data)
    label = Int16()
    # print(submap.data)



    submap.data = np.array(submap.data).reshape(16, 16)
    submap.data = gf.nullfix(submap.data)
    submap.data = gf.shaping(submap.data)
    submap.data = np.array(submap.data).reshape(1,1,16,16)
    b=torch.Tensor(submap.data)
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
    predic = torch.max(trueresult,1)
    t2=time.time()
    #predic = int(predic)
    label.data = predic
  # label_pub.publish(trueresult)
    print(predic)
    print(trueresult)
    print(t2-t1)

if __name__ == '__main__':
    rospy.init_node('CNN')
    rospy.Subscriber('/submap_image', Image, predict_callback)
    label_pub = rospy.Publisher('CNN_Foothold', Int16, queue_size=10)
    #sys.stdout = open('output.txt','w')
    rospy.spin()
