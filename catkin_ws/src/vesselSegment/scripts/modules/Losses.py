import torch
import numpy as np

def DiceLoss(pred,target,smooth=0):
#     print("pred shape: ",pred.shape)
#     print("target shape: ",target.shape)
    
#     print(torch.unique(target))
    target = target/255
#     print(torch.unique(target))
    index = (2*torch.sum(pred*target)+smooth)/(torch.sum(pred)+torch.sum(target)+smooth)
#     print("torch.sum(pred*target): ",2*torch.sum(pred*target))
#     print("torch.sum(pred): ",torch.sum(pred))
#     print("torch.sum(target): ",torch.sum(target))
    #if torch.sum(target).item() == 0:
    #print("instersection: ",torch.sum(pred*target).item())
    # print("pred: ",torch.sum(pred).item())
    # print("target: ",torch.sum(target).item())
    #print("Index: ", index.item())
    return 1-index