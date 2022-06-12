import cv2
import os
import numpy as np
from numpy import average
import torchvision

# fileDice=open("jbl0170Diceunet.txt","a")
# filePrecision=open("jbl0170Precisionunet.txt","a")
# fileRecall=open("jbl0170Recallunet.txt","a")
mask_ground="/home/zhongliang/yuangao/download/FL0167"
mask_pred="/home/zhongliang/yuangao/roscode/catkin_ws/src/vesselSegment/models/unet/label67"
filePaths1=os.listdir(mask_pred)
filePaths1=sorted(filePaths1)
filePaths2=os.listdir(mask_ground)
filePaths2=sorted(filePaths2)
dice_SUM=0
accuracy_sum=0
precision_sum=0
recall_sum=0
num_sum=0
n=0
i=0
num=0
for entry1 in filePaths1:
    if n<=0:
        n+=1
        continue
    if n>250:
        break
    
    mask_G=cv2.imread(os.path.join(mask_ground,filePaths2[n]))
    print(os.path.join(mask_ground,filePaths2[n]))
    mask_G=mask_G[60:658,465:1136]
    _,mask_G=cv2.threshold(mask_G,0.5,255,cv2.THRESH_BINARY)
    cv2.imwrite("/home/zhongliang/yuangao/download/FL01672/"+str(n)+".png",mask_G)
    mask_P=cv2.imread(os.path.join(mask_pred,str(n)+".png"))
    print(os.path.join(mask_pred,str(n)+".png"))
    _,mask_P=cv2.threshold(mask_P,1,255,cv2.THRESH_BINARY)   
    # cv2.imshow("mask_G", mask_G)
    # cv2.imshow("mask_P", mask_P)
    num=len(np.where(mask_P==255)[0])-len(np.where(mask_G==255)[0])
    jiaoji=cv2.bitwise_and(mask_P, mask_G)
    # cv2.imshow("jiaoji", jiaoji)
    TP=len(np.where(jiaoji==255)[0])
    print(TP)
    bingji=cv2.bitwise_or(mask_P, mask_G)
    # cv2.imshow("bingji", bingji)
    TN=671*598-len(np.where(bingji==255)[0])
    print(TN)
    pchaji=cv2.bitwise_xor(mask_P, jiaoji)
    # cv2.imshow("pchaji", pchaji)
    gchaji=cv2.bitwise_xor(mask_G, jiaoji)
    # cv2.imshow("gchaji", gchaji)
    FP=len(np.where(pchaji==255)[0])
    print(FP)
    FN=len(np.where(gchaji==255)[0])
    print(FN)
    accuracy_score=(TP+TN)/(TP+TN+FP+FN)
    precision=TP/(TP+FP)
    recall=TP/(TP+FN)
    dice = np.sum(mask_P[mask_G == 255]) * 2.0 / (np.sum(mask_G) + np.sum(mask_P))
    # fileDice.writelines('\n'+str(dice))
    # filePrecision.writelines('\n'+str(precision))
    # fileRecall.writelines('\n'+str(recall))
    print("dice:%0.3f"%dice)
    print("accuracy_score:%0.3f"%accuracy_score)
    print("precision:%0.3f"%precision)
    print("recall:%0.3f"%recall)
    dice_SUM+=dice
    accuracy_sum+=accuracy_score
    precision_sum+=precision
    recall_sum+=recall
    num_sum+=num
    n=n+1
    # cv2.waitKey(0)
print("average dice: %0.3f"%(dice_SUM/(250)))
print("average accuracy: %0.3f"%(accuracy_sum/(250)))
print("average precision: %0.3f"%(precision_sum/(250)))
print("average recall: %0.3f"%(recall_sum/(250)))
print("average num: %0.3f"%(num_sum/(250)))

#of-unet
#Feng Li index:0167 1-250 250pics  dice:0.843 precision: 0.815 recall: 0.881
#Bailiang index:0170 451-700 250pics   dice:0.856 precision:0.783 recall:0.945

#unet
#Feng Li index:0167 1-250 250pics   dice:0.799 precision: 0.759 recall:0.854
#Bailiang index:0170 451-700 250pics   dice:0.792 precision:0.720 recall:0.898