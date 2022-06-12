import cv2
import os
import numpy as np

# fusionDir="/home/zhongliang/yuangao/roscode/catkin_ws/src/vesselSegment/models/OFunet/fusion77"
fusionDir="/home/zhongliang/yuangao/roscode/catkin_ws/src/vesselSegment/models/OFunet/fusion70"
filePaths=os.listdir(fusionDir)
n=len(filePaths)
size = (671, 598)

fourcc = cv2.VideoWriter_fourcc('M','J','P','G') #opencv3.0
videoWrite = cv2.VideoWriter( '70.avi', fourcc, 50, size )
# 写入对象 1 file name 2 编码器 3 帧率 4 尺寸大小

for i in range(n-1, 1, -1):
    fileName = os.path.join(fusionDir,str(i)+'.png')
    img = cv2.imread(fileName)
    imgInfo = img.shape
    videoWrite.write(img) # 写入方法 1 jpg data
videoWrite.release()
print('end')