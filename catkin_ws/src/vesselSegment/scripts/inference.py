import rospy
from sensor_msgs.msg import Image as SensorImage
import torch
import torch.nn as nn
import torchvision.transforms as transforms

from modules.UNet import *
from modules.Discriminator import *
from modules.DataSet import *
from modules.Losses import *
from torchvision.transforms import InterpolationMode

from flownet2 import models

import os,sys

from torch.utils.data import Dataset, DataLoader
from torch.utils.data.dataset import random_split
from PIL import Image,ImageEnhance

import numpy as np
from inferenceOF import *
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32MultiArray
import time

OFNet=inferenceWithOF()
bridge=CvBridge()
# pilImg=Image.open("30000.png")
pub = rospy.Publisher('/vessel/position', Float32MultiArray, queue_size=1)
f=open("55.txt","a")

def segment(rosimage):
    # global pilImg

    cvImg=bridge.imgmsg_to_cv2(rosimage)
    cropped=cvImg[60:658,465:1136] #ifl's device
    # cropped=cvImg[121:810,584:1351] #isar's device
    pilImg=Image.fromarray(cropped)
    bool_value = rospy.get_param("p_bool")
    milliseconds1 = int(round(time.time() * 1000))
    # print(milliseconds1)

    label=OFNet.inference(pilImg,bool_value)

    milliseconds2 = int(round(time.time() * 1000))
    # print(milliseconds2-milliseconds1)

    # label.save("dd.png")
    cvlabel=cv2.cvtColor(np.asarray(label),cv2.COLOR_RGB2BGR)
    img_gray = cv2.cvtColor(cvlabel,cv2.COLOR_RGB2GRAY)
    contours, _ = cv2.findContours(img_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    M1=cv2.moments(contours[0])
    cx=int(M1["m10"] / M1["m00"])
    cy=int(M1["m01"] / M1["m00"])
    # array=[cx/767,cy/689] #isar's device
    array=[cx/671,cy/598] #ifl's device
    position=Float32MultiArray(data=array)


    pub.publish(position)
    rospy.loginfo("vessel's position: %0.2f %0.2f",position.data[0],position.data[1])

def segmentLocal():
    image_path="/home/zhongliang/yuangao/download/0170"
    filePaths=os.listdir(image_path)
    filePaths=sorted(filePaths)
    for entry in filePaths:
        # if int(entry[0:5])<73400 or int(entry[0:5])>74000:
            # continue
        # print(os.path.join(image_path,str(entry)))
        img=cv2.imread(os.path.join(image_path,str(entry)),0)
        print(os.path.join(image_path,str(entry)))
        rospy.loginfo("sending pics:%s",entry)

        cropped=img[60:658,465:1136] #ifl's device
        # cropped=cvImg[121:810,584:1351] #isar's device
        pilImg=Image.fromarray(cropped)
        contrast_enhancer = ImageEnhance.Contrast(pilImg)
        # 传入调整系数1.2
        contrast_img = contrast_enhancer.enhance(1.2)

        bool_value = rospy.get_param("p_bool")
        milliseconds1 = int(round(time.time() * 1000))
        # print(milliseconds1)

        label=OFNet.inference(contrast_img,bool_value)

        milliseconds2 = int(round(time.time() * 1000))
        # print(milliseconds2-milliseconds1)

        # label.save("dd.png")
        cvlabel=cv2.cvtColor(np.asarray(label),cv2.COLOR_RGB2BGR)
        img_gray = cv2.cvtColor(cvlabel,cv2.COLOR_RGB2GRAY)
        contours, _ = cv2.findContours(img_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        M1=cv2.moments(contours[0])
        cx=int(M1["m10"] / M1["m00"])
        cy=int(M1["m01"] / M1["m00"])
        # array=[cx/767,cy/689] #isar's device
        array=[cx/671,cy/598] #ifl's device
        position=Float32MultiArray(data=array)
        f.writelines('\n'+str(array[0]))
        rospy.loginfo("vessel's position: %0.2f %0.2f",position.data[0],position.data[1])

# if __name__=="__main__":
#     rospy.init_node("segmentVessel")
#     sub=rospy.Subscriber("/imfusion/cephasonics",SensorImage,segment,queue_size=1)
#     rospy.loginfo("start segmentation...")
#     rospy.spin()

if __name__=="__main__":
    rospy.init_node("segmentVessel")
    segmentLocal()

