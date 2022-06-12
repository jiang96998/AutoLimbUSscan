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

from inferenceUnet import *

import numpy as np


class FlowNet2Args():
    fp16 = False
    rgb_max = 255
    
class inferenceWithOF:
    def __init__(self):
        self.__device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.__PATH = "../models/unet_of_usseg_humanOF.pth"
        self.__unet = UNet_OF2(init_features=64).to(self.__device)
        self.__unet.load_state_dict(torch.load(self.__PATH))
        self.__unet = self.__unet.eval()
        self.__img_size=[256,256]
        self.__args = FlowNet2Args()
        self.__flownet = models.FlowNet2(self.__args).cuda()
        self.__flownet.load_state_dict(torch.load('../models/FlowNet2_checkpoint.pth.tar')["state_dict"])
        self.__flownet = self.__flownet.eval()
        self.__unet = self.__unet.eval()
        self.__transform_image = transforms.Compose([
                transforms.ToTensor(),
                transforms.Normalize(0.5,0.5)
            ])
        self.__invtransform_label = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize([598,671],interpolation=InterpolationMode.NEAREST) #ifl's device
            # transforms.Resize([689,767],interpolation=InterpolationMode.NEAREST) #isar's device
            ])
        self.pureUnet=inferenceWithUnet()
        self.__pureUnet=True
        self.index=0
        self.fileIndex=70

    def __get_next_mask(self,label,flow):
        #print(flow.shape)
        #print(label.shape)
        label = np.array(label)
        result = np.zeros_like(label)            
        rows,cols = np.where(label>0.5)

        for i in range(len(cols)):                
            [dcol,drow] = flow[:,rows[i],cols[i]]                
            tmp_row = np.clip(int(rows[i]+drow),0,label.shape[0]-1)
            tmp_col = np.clip(int(cols[i]+dcol),0,label.shape[1]-1)                
            result[tmp_row,tmp_col] = 1
        
        return result

    def inference(self,img,ispureUnet=True):
        # if self.__pureUnet:
        if ispureUnet:
            # self.__pureUnet=ispureUnet
            self.__preimg=img        
            self.__prelabel=self.pureUnet.inference(img)
            # print(self.__preimg.format,self.__preimg.size)

            mmm=Image.blend(self.__preimg,self.__prelabel,0.5)
            sav_path1 ="../models/OFunet/fusion"+str(self.fileIndex)+"/"+str(self.index)+".png"
            self.__preimg.save(sav_path1)
            sav_path2 ="../models/OFunet/label"+str(self.fileIndex)+"/"+str(self.index)+".png"
            self.__prelabel.save(sav_path2)
            self.index+=1
            return self.__prelabel
        else:
            self.__preimg=self.__preimg.resize(self.__img_size,resample=Image.NEAREST)
            img2=img.resize(self.__img_size,resample=Image.NEAREST)
            img_ = np.array(img2.convert('RGB'))
            preimg_ = np.array(self.__preimg.convert('RGB'))
            im=np.ascontiguousarray(np.array([preimg_,img_]).transpose(3, 0, 1, 2))
            im = torch.from_numpy(im.astype(np.float32))
            im=im.unsqueeze(0)
            # print(im.shape)
            # print(img_.shape)
            #calculate optical flow
            im = im.to(self.__device)
            with torch.no_grad():
                flow = self.__flownet(im)
            flow = np.array(flow.cpu())
            labels_curr_of = np.zeros([1,1,256,256])
            # print(labels_curr_of.shape)
            self.__prelabel=self.__prelabel.resize(self.__img_size,resample=Image.NEAREST)
            self.__prelabel=np.array(self.__prelabel)
            self.__prelabel=np.expand_dims(self.__prelabel,axis=0)
            self.__prelabel=np.expand_dims(self.__prelabel,axis=0)
            # print(self.__prelabel.shape)
            labels_curr_of[0,...] = self.__get_next_mask(self.__prelabel[0,0,...],flow[0,...])
            labels_curr_of = torch.Tensor(labels_curr_of).to(self.__device,dtype=torch.float32)            
            img2 = self.__transform_image(img2)
            img2 = img2.unsqueeze(0)
            img2 = img2.to(self.__device)
            # print(img2.shape)
            # milliseconds1 = int(round(time.time() * 1000))
            # print(milliseconds1)
            with torch.no_grad():
                pred = self.__unet(img2,labels_curr_of,mode=1)
                    
            self.__prelabel =self.__invtransform_label(pred.cpu().squeeze(0))
            # print(self.__prelabel.shape)
            self.__preimg=img

            # milliseconds2 = int(round(time.time() * 1000))
            # print(milliseconds2-milliseconds1) 
            mmm=Image.blend(self.__preimg,self.__prelabel,0.5)
            sav_path1 ="../models/OFunet/fusion"+str(self.fileIndex)+"/"+str(self.index)+".png"
            self.__preimg.save(sav_path1)
            sav_path2 ="../models/OFunet/label"+str(self.fileIndex)+"/"+str(self.index)+".png"
            self.__prelabel.save(sav_path2)
            # print(sav_path2)
            self.index+=1
            return self.__prelabel
