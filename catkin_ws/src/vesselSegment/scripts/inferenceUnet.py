#!/usr/bin/env python
# coding: utf-8

import torch
import torch.nn as nn
import torchvision.transforms as transforms
from torchvision.transforms import InterpolationMode

from modules.UNet import *
from modules.Discriminator import *
from modules.DataSet import *
from modules.Losses import *

import os,sys

from torch.utils.data import Dataset, DataLoader
from torch.utils.data.dataset import random_split
from PIL import Image,ImageEnhance

import numpy as np
import cv2

import time
import matplotlib.pyplot as plt

class inferenceWithUnet:
    def __init__(self):
        self.__device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.__PATH = os.path.expanduser("../models/unet_usseg_human_withoutOF.pth")
        self.__unet = UNet(init_features=64).to(self.__device)
        self.__unet.load_state_dict(torch.load(self.__PATH))
        self.__unet = self.__unet.eval()

        self.__resize_to=[256,256]
        self.__transform_image = transforms.Compose([
            transforms.Resize(self.__resize_to,interpolation=InterpolationMode.NEAREST),
            transforms.ToTensor(),
            transforms.Normalize(0.5,0.5) #Division by 255 is done, when the transformation assumes an image.
            ])
        self.__invtransform_label = transforms.Compose([
            transforms.ToPILImage(),
        #     transforms.Resize([[549,690]])
            transforms.Resize([598,671],interpolation=InterpolationMode.NEAREST) #ifl's device
            # transforms.Resize([689,767],interpolation=InterpolationMode.NEAREST) #isar's device
            ])

    def inference(self,img):


        # milliseconds1 = int(round(time.time() * 1000))
        # print(milliseconds1)
        img = self.__transform_image(img).unsqueeze(0)
        img = img.to(self.__device)
        # milliseconds2 = int(round(time.time() * 1000))
        # print(milliseconds2-milliseconds1)
        # milliseconds1 = int(round(time.time() * 1000))
        # print(milliseconds1)
        with torch.no_grad():
            pred = self.__unet(img)

        # milliseconds2 = int(round(time.time() * 1000))
        # print(milliseconds2-milliseconds1)
        pred = self.__invtransform_label(pred.cpu().squeeze(0))
        # milliseconds3 = int(round(time.time() * 1000))
        # print(milliseconds3-milliseconds1)
        return pred

# a=inferenceWithUnet()
# image_path = "/home/y/RosCode/catkin_ws/30000.png"
# img = Image.open(image_path).convert("L")

# b=a.inference(img)
# sav_path ="../models/pred.png"
# # print(b)
# b.save(sav_path)
