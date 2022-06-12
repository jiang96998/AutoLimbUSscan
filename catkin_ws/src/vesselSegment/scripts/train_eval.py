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




def train(net,device,val_per=0.1,epochs=10,batch_size=10,resize_to=None):
    if resize_to is not None:
        transform_image = transforms.Compose([
        transforms.Resize(resize_to),
        transforms.ToTensor(),
        transforms.Normalize(0.5,0.5)
        ])
        transform_label = transforms.Compose([
        transforms.Resize(resize_to,interpolation=InterpolationMode.NEAREST),
        #transforms.ToTensor()
        ])
    else:
        transform_image = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(0.5,0.5)
        ])
        transform_label = transforms.Compose([
            
        #transforms.ToTensor()
        ])
    
    dataSet = UltraSoundDataSet2(root_dir,[transform_image,transform_label])
    nTrain = int(len(dataSet)*(1-val_per))
    nValid = int(len(dataSet)-nTrain)
    
    trainSet,validSet = random_split(dataSet,[nTrain,nValid])
    
    train_loader = DataLoader(trainSet,batch_size=batch_size,shuffle=True,num_workers=4)
    valid_loader = DataLoader(validSet,batch_size=batch_size,shuffle=True,num_workers=4)
    
    optimizer = torch.optim.Adam(net.parameters())
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer,mode='min',patience=10)
    
    running_loss_seg = 0
    
    step = 0
    np.set_printoptions(precision=2)
    
    for epoch in range(epochs):
        net.train()
        
        for batch in train_loader:
            imgs,labels = batch

            images = imgs.to(device=device,dtype=torch.float32)
            labels = labels.to(device=device,dtype=torch.float32)
            
            pred = net(images)
                
            seg_loss = DiceLoss(pred,labels)
            
            #print(seg_loss.item())
            optimizer.zero_grad()
            seg_loss.backward()
            optimizer.step()

            running_loss_seg += seg_loss.item()
            
            step += 1    
            if step % 10 == 9:    # print every 10 mini-batches
                print()
                print('[%d, %5d] loss: %.3f' %(epoch + 1, step + 1, running_loss_seg / 10))
                running_loss_seg = 0.0
                
            if step%50 == 49:
                net.eval()
                val_loss = 0
                for batch in valid_loader:
                    imgs,labels = batch

                    labels = labels.to(device)
                    images = imgs.to(device)
                    with torch.no_grad():
                        pred = net(images)

                    val_loss += DiceLoss(pred,labels)
                print('[%d, %5d] validation loss: %.3f' %(epoch + 1, step + 1, val_loss / len(valid_loader)))
                scheduler.step(val_loss)
                net.train()




device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
unet = UNet(init_features=64).to(device)
#root_dir = os.path.expanduser("~/workspace/us_robot/DataSet/realDataSet/linear/vessel_clip/vessel_dataset")
root_dir = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/4")




try:
    train(unet,device,val_per=0.1,epochs=5,batch_size=10,resize_to=(256,256))
except KeyboardInterrupt:
    sys.exit()



#for python
torch.save(unet.state_dict(), '/home/robotics-meta/Project/yuanGao/trainData/train/4/unet_usseg_human_withoutOF_cropped4.pth')
# #for c++
# traced_script_module = torch.jit.trace(unet, img)
# traced_script_module.save("./unet_of_usseg_traced.pt")


# ## Inference


device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

PATH = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/4/unet_usseg_human_withoutOF_cropped4.pth")
unet = UNet(init_features=64).to(device)
unet.load_state_dict(torch.load(PATH))
unet = unet.eval()







test_dir = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/4/test/image")
label_dir = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/4/test/mask")
pred_dir = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/4/test/mask_pred")

testset_list = os.listdir(test_dir)
testset_list = sorted(list(filter(lambda x: x.endswith('png'), testset_list)))
resize_to=[256,256]
transform_image = transforms.Compose([
    transforms.Resize(resize_to,interpolation=InterpolationMode.NEAREST),
    transforms.ToTensor(),
    transforms.Normalize(0.5,0.5) #Division by 255 is done, when the transformation assumes an image.
    ])

invtransform_label = transforms.Compose([
    transforms.ToPILImage(),
#     transforms.Resize([[549,690]])
    transforms.Resize([549,690],interpolation=InterpolationMode.NEAREST)
    ])

# run_cntr = 1
# avg_dt = 0
# avg_DiceIdx = 0

hist_dt = []
hist_dice = []
for sample in testset_list:
    image_path = os.path.join(test_dir,sample)
    label_path = os.path.join(label_dir,sample)
    
    img = Image.open(image_path).convert("L")
    #img = ImageEnhance.Contrast(img).enhance(1.5)
    img = transform_image(img).unsqueeze(0)
#     labels_curr_of = torch.Tensor(np.zeros_like(img))
    img = img.to(device)
#     labels_curr_of = labels_curr_of.to(device)
    label = torch.Tensor(np.array(Image.open(label_path))).to(device)
    label = label.unsqueeze(0)
    
    ti = time.time()
    with torch.no_grad():
#         pred = unet(img,labels_curr_of, mode=0)
        pred = unet(img)
    
    dt = time.time()-ti
#     avg_dt = (run_cntr-1)/run_cntr*avg_dt+1/run_cntr*dt
    
#     print("avg pred time: ",dt)
    pred = invtransform_label(pred.cpu().squeeze(0))
    sav_path = os.path.join(pred_dir,sample)
    pred.save(sav_path)
    '''
    DiceIndex = (1 - DiceLoss(pred,label)).cpu().item()
#     avg_DiceIdx = (run_cntr-1)/run_cntr*avg_DiceIdx+1/run_cntr*DiceIndex
#     print("avg pred Dice Coeff: ",avg_DiceIdx)
#     run_cntr += 1
    
    hist_dt += [dt]
    hist_dice += [DiceIndex]

#     pred = invtransform_label(pred.cpu().squeeze(0))
#     #fname = "pred%.2f.png"%DiceIndex
#     fname = sample
#     sav_path = os.path.join(pred_dir,fname)
#     pred.save(sav_path)
#     print(pred.shape)
    pred = pred.cpu()
    pred = np.array(pred[0].permute(1, 2, 0))
    pred = (pred*255).astype(np.uint8)
    _,pred = cv2.threshold(pred,thresh=127,maxval=255,type=cv2.THRESH_BINARY)
    
    
#     print(label.shape)
    
    label = label.cpu()
    label = np.array(label.permute(1, 2, 0))
    label = (label*255).astype(np.uint8)
    _,label = cv2.threshold(label,thresh=127,maxval=255,type=cv2.THRESH_BINARY)
    
    img_rgb = cv2.cvtColor(np.array((np.squeeze(img.cpu())/2+0.5)*255),cv2.COLOR_GRAY2RGB)
#     pred_rgb = cv2.cvtColor(pred,cv2.COLOR_GRAY2RGB)
    
#     pred_rgb[:,:,-2] = 0
    
    contours, _ = cv2.findContours(pred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img_rgb, contours, -1, (200,0,0), 1)
    
    contours, _ = cv2.findContours(label, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img_rgb, contours, -1, (0,200,0), 1)
    
    fname = sample+"_%.2f.png"%DiceIndex
    sav_path = os.path.join(pred_dir,fname)
    
#     resultImg = Image.fromarray( (pred_rgb*0.2+img_rgb).astype(np.uint8) )
#     resultImg = Image.fromarray( img_rgb )

#     resultImg.save(sav_path)
    plt.imsave(sav_path,img_rgb/255)
    '''



print(np.mean(hist_dice))
print(max(hist_dice) - np.mean(hist_dice))
print(min(hist_dice) - np.mean(hist_dice))

print(np.mean(hist_dt))
print(max(hist_dt) - np.mean(hist_dt))
print(min(hist_dt) - np.mean(hist_dt))


import matplotlib.pyplot as plt
plt.imshow(img_rgb/255)
plt.show()
