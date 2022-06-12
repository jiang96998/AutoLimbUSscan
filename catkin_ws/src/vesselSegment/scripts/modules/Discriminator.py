import torch
import torch.nn as nn
import torch.nn.functional as F

class Discriminator(nn.Module):
    def __init__(self):
        super(Discriminator, self).__init__()
        
        self.branch1 = Discriminator.image_branch()
        self.branch2 = Discriminator.mask_branch()
        
        self.dis_conv1 = nn.Conv2d(128, 256, kernel_size = 3, padding=1)
        self.dis_pool1 = nn.MaxPool2d(2, 2)
        self.dis_conv2 = nn.Conv2d(256, 512, kernel_size = 3, padding=1)
        #self.dis_pool2 = nn.MaxPool2d(2, 2)
        
        self.output = nn.Sequential(
                        nn.Linear(512,128),
                        nn.ReLU(inplace=True),
                        nn.Linear(128,1)
                        )
        
        
    def forward(self, input, mask):
        x = self.branch1(input)
        y = self.branch2(mask)
        
        x = torch.cat([x,y],dim=1)
        x = F.relu(self.dis_conv1(x))
        x = self.dis_pool1(x)
        x = F.relu(self.dis_conv2(x))
        #x = self.dis_pool2(x)
        x = F.max_pool2d(x,kernel_size=x.size()[2:]).squeeze() #global max pooling

        output = torch.sigmoid(self.output(x))

        return output

    def image_branch():
        return nn.Sequential(
                nn.Conv2d(1, 16, kernel_size = 3, padding=1),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2, 2),
                nn.Conv2d(16, 64, kernel_size = 3, padding=1),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2, 2)
            )
    def mask_branch():
        return nn.Sequential(
                nn.MaxPool2d(2, 2),
                nn.Conv2d(1, 64, kernel_size = 3, padding=1),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2, 2)
            )