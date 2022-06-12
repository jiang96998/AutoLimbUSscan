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

def get_next_mask_edge(label,flow):
    #print(flow.shape)
    #print(label.shape)
    label = np.array(label)
    edge_points = []
    result = np.zeros_like(label)
    for i in range(label.shape[0]):
        curr_row = label[i,:]
        if any(curr_row>0):
            lnon_zero = np.where(curr_row>0)[0]
            edge_points.append([i,lnon_zero[0]])
            edge_points.append([i,lnon_zero[-1]])

    for j in range(label.shape[1]):
        curr_col = label[:,j]
        if any(curr_col>0):
            lnon_zero = np.where(curr_col>0)[0]
            edge_points.append([lnon_zero[0],j])
            edge_points.append([lnon_zero[-1],j])

    for coord in edge_points:
        [dx,dy] = flow[:,coord[0],coord[1]]
        coord = (coord+np.array([dy,dx])).astype(np.long)
        coord[0] = np.clip(coord[0],0,label.shape[0]-1)
        coord[1] = np.clip(coord[1],0,label.shape[1]-1)
        
        result[coord[0],coord[1]] = 1
    
    return result

def get_next_mask(label,flow):
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

def train_OF(net,flownet,device,val_per=0.1,epochs=10,batch_size=10,resize_to=None):

    transform_image = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(0.5,0.5)
    ])

        
    dataSet = UltraSoundDataSet_OF(root_dir,transform_image,resize_to)
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
            im, images_curr, labels_prev, labels_curr = batch
                
            #calculate optical flow
            im = im.to(device)
            with torch.no_grad():
                flow = flownet(im)

            flow = np.array(flow.cpu())
            
            labels_curr_of = np.zeros_like(labels_curr)
            for n in range(im.shape[0]):
                #estimate vessel position by using the flow
                labels_curr_of[n,...] = get_next_mask(labels_prev[n,0,...],flow[n,...])
                
            labels_curr_of = torch.Tensor(labels_curr_of).to(device,dtype=torch.float32)
            images_curr = images_curr.to(device)
            labels_curr = labels_curr.to(device)
#             print(images_curr.shape)
#             print(labels_curr_of.shape)
            if np.random.rand(1)>0.5:
                pred = net(images_curr,labels_curr_of, mode=1)
                print("+",end='')
            else:
                pred = net(images_curr,labels_curr_of, mode=0)
                print("-",end='')
                
            seg_loss = DiceLoss(pred,labels_curr)
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
                    im, images_curr, labels_prev, labels_curr = batch
                    
                    #calculate optical flow
                    im = im.to(device)
                    with torch.no_grad():
                        #print(im.shape)
                        flow = flownet(im)
                        #print(flow.shape)

                    flow = np.array(flow.cpu())
                        
                    labels_curr_of = np.zeros_like(labels_curr)
                    for n in range(im.shape[0]):
                        #estimate vessel position by using the flow   
                        labels_curr_of[n,...] = get_next_mask(labels_prev[n,0,...],flow[n,...])
                    
                    labels_curr_of = torch.Tensor(labels_curr_of).to(device,dtype=torch.float32)
                    labels_curr = labels_curr.to(device)
                    images_curr = torch.Tensor(images_curr).to(device)
                    with torch.no_grad():
                        pred = net(images_curr,labels_curr_of,mode=0)

                    val_loss += DiceLoss(pred,labels_curr)
                print('[%d, %5d] validation loss: %.3f' %(epoch + 1, step + 1, val_loss / len(valid_loader)))
                scheduler.step(val_loss)
                net.train()

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
#print("use ",device)
unet = UNet_OF2(init_features=64).to(device)

class FlowNet2Args():
    fp16 = False
    rgb_max = 255

args = FlowNet2Args()
flownet = models.FlowNet2(args).cuda()
flownet.load_state_dict(torch.load(os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/jupyterProject/FlowNet2_checkpoint.pth.tar"))["state_dict"])
flownet = flownet.eval()

#IMG_SIZE = [256,256]

# root_dir = os.path.expanduser("~/workspace/us_robot/DataSet/realDataSet/linear/vessel_dataset")
# root_dir = os.path.expanduser("~/workspace/us_robot/DataSet/phantomDataset/vessel_dataset")
root_dir = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/3")


try:
    train_OF(unet,flownet,device,val_per=0.1,epochs=10,batch_size=10,resize_to=(256,256))
except KeyboardInterrupt:
    sys.exit()

#for python
torch.save(unet.state_dict(), "/home/robotics-meta/Project/yuanGao/trainData/train/3/unet_of_usseg_human_cropped.pth")
# #for c++
# traced_script_module = torch.jit.trace(unet, img)
# traced_script_module.save("./unet_of_usseg_traced.pt")

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

PATH = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/3/unet_of_usseg_human_cropped.pth")
unet = UNet_OF2(init_features=64).to(device)
unet.load_state_dict(torch.load(PATH))
unet = unet.eval()

class FlowNet2Args():
    fp16 = False
    rgb_max = 255

args = FlowNet2Args()
flownet = models.FlowNet2(args).cuda()
flownet.load_state_dict(torch.load('/home/robotics-meta/Project/yuanGao/trainData/jupyterProject/FlowNet2_checkpoint.pth.tar')["state_dict"])
flownet = flownet.eval()
unet = unet.eval()
flownet = flownet.eval()

test_dir = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/3/test")
pred_dir = os.path.expanduser("/home/robotics-meta/Project/yuanGao/trainData/train/3/test/mask_pred")

transform_image = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(0.5,0.5)
    ])
invtransform_label = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize([549,690],interpolation=InterpolationMode.NEAREST)
    ])

eval_dataSet = UltraSoundDataSet_OF(test_dir,transform_image,resize_to=[256,256])
# eval_dataSet = UltraSoundDataSet_OF(test_dir,transform_image)
eval_loader = DataLoader(eval_dataSet,batch_size=1,shuffle=False,num_workers=4)

eval_loss = 0
for idx,batch in enumerate(eval_loader):
    im, images_curr, labels_prev_init, labels_curr = batch
    
    #calculate optical flow
    im = im.to(device)
    with torch.no_grad():
        flow = flownet(im)

    flow = np.array(flow.cpu())

    labels_curr_of = np.zeros_like(labels_curr)
    print(labels_curr_of.shape)
    print(labels_prev_init.shape)
    print(flow.shape)
    #estimate vessel position by using the flow
    if idx == 0:
        labels_curr_of[0,...] = get_next_mask(labels_prev_init[0,0,...],flow[0,...])
    else:        
        labels_curr_of[0,...] = get_next_mask(labels_prev_init[0,0,...],flow[0,...])

    labels_curr_of = torch.Tensor(labels_curr_of).to(device,dtype=torch.float32)
    labels_curr = labels_curr.type(torch.float32).to(device)
    images_curr = images_curr.to(device)
    
    with torch.no_grad():
        pred = unet(images_curr,labels_curr_of,mode=int(idx>0))
        #pred = unet(images_curr,labels_curr_of,mode=0)
    
    eval_loss += DiceLoss(pred,labels_curr)
    
    labels_prev = pred.cpu()
    
    pred = invtransform_label(pred.cpu().squeeze(0))
    #fname = "pred%.2f.png"%DiceIndex
    fname = "%d.png" %(idx+30000)
    sav_path = os.path.join(pred_dir,fname)
    pred.save(sav_path)
    
print('evaluation loss: %.3f' %(eval_loss / len(eval_loader)))
test_dir = os.path.expanduser("~/workspace/us_robot/DataSet/realDataSet/linear/vessel_dataset")
transform_image = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(0.5,0.5)
    ])

eval_dataSet = UltraSoundDataSet_OF(test_dir,transform_image)
eval_loader = DataLoader(eval_dataSet,batch_size=1,shuffle=False,num_workers=4)
batch = next(iter(eval_loader))
im, images_curr, labels_prev_init, labels_curr = batch