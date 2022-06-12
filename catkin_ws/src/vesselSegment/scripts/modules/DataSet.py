import torch
from torch.utils.data import Dataset
from PIL import Image
import numpy as np
import os
from torchvision.transforms import InterpolationMode


class UltraSoundDataSet(Dataset):
    def __init__(self, root_dir, transforms):
        self.root_dir = root_dir
        self.sample_list = os.listdir(root_dir)
        self.transform_image, self.transform_label = transforms
        
    def __len__(self):
        return len(self.sample_list)#返回样本数量
    
    def __getitem__(self,idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        image_path = os.path.join(self.root_dir,self.sample_list[idx],"image.png")
        label_path = os.path.join(self.root_dir,self.sample_list[idx],"label.png")
        
        image = Image.open(image_path)
        label = Image.open(label_path)
        
        if self.transform_image is not None:
            image = self.transform_image(image)
            
        if self.transform_label is not None:
            label = self.transform_label(label)
        
        return image,label

class UltraSoundDataSet2(Dataset):
    def __init__(self, root_dir, transforms):
        self.root_dir = root_dir
        self.sample_list = os.listdir(root_dir+'/image')
        self.transform_image, self.transform_label = transforms
        
    def __len__(self):
        return len(self.sample_list)
    
    def __getitem__(self,idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        image_path = os.path.join(self.root_dir+'/image/',self.sample_list[idx])
        label_path = os.path.join(self.root_dir+'/mask/',self.sample_list[idx])
        
        image = Image.open(image_path)
        label = Image.open(label_path)
        
        if self.transform_image is not None:
            image = self.transform_image(image)
            
        if self.transform_label is not None:
            label = self.transform_label(label)
        label = torch.Tensor(np.array(label)).unsqueeze(0)
        return image,label

class UltraSoundDataSet_OF(Dataset):
    def __init__(self, root_dir, transform_image=None, resize_to=None):
        import yaml
        self.root_dir = root_dir
        config = yaml.load(open(root_dir+'/config.yaml'),Loader=yaml.FullLoader)
        print(config)
        first_frames = config.get('stream_begin',[])

        tmp = os.listdir(root_dir+'/image')
        tmp = sorted(list(filter(lambda x: x.endswith('png'), tmp)))
        #remove all first frames to ensure there is always a previous frame
        for ff in first_frames:
            file_name = ff
            tmp.remove(file_name)

        self.sample_list = tmp
        self.transform_image = transform_image
        self.img_size = resize_to
        
    def __len__(self):
        return len(self.sample_list)
    
    def __getitem__(self,idx):
        FLAG_INIT = False
        if torch.is_tensor(idx):
            idx = idx.tolist()

        #read current image-label pair
        curr_img_fname = self.sample_list[idx]  
#         print(curr_img_fname)
        numberpath,_=os.path.splitext(curr_img_fname)    
#         print(numberpath)
        ser_num=int(numberpath)
#         ser_num = float(curr_img_fname[0:5])
#         print(ser_num)

        curr_image_path = os.path.join(self.root_dir,'image',self.sample_list[idx])
        curr_label_path = os.path.join(self.root_dir,'mask',self.sample_list[idx])
        image_curr = Image.open(curr_image_path).convert('L')
        label_curr = Image.open(curr_label_path).convert('L')

        #read previous image-label pair 
        prev_image_path = os.path.join(self.root_dir,'image',"%d.png" %(ser_num-1))
        prev_label_path = os.path.join(self.root_dir,'mask',"%d.png" %(ser_num-1))
        image_prev = Image.open(prev_image_path).convert('L')
        label_prev = Image.open(prev_label_path).convert('L')

        if self.img_size is not None:
            image_curr = image_curr.resize(self.img_size,resample=Image.NEAREST)
            label_curr = label_curr.resize(self.img_size,resample=Image.NEAREST)
            image_prev = image_prev.resize(self.img_size,resample=Image.NEAREST)
            label_prev = label_prev.resize(self.img_size,resample=Image.NEAREST)

        #make image pair for FlowNet2
        image_curr_ = np.array(image_curr.convert('RGB'))
        image_prev_ = np.array(image_prev.convert('RGB'))
        im = np.ascontiguousarray(np.array([image_prev_,image_curr_]).transpose(3, 0, 1, 2))
        im = torch.from_numpy(im.astype(np.float32))

        if self.transform_image is not None:
            image_curr = self.transform_image(image_curr)

        label_prev = np.array(label_prev)
        label_prev = np.expand_dims(label_prev, axis=0)

        label_curr = np.array(label_curr)
        label_curr = np.expand_dims(label_curr, axis=0)
        
        return im, image_curr, label_prev, label_curr
        # im: Tensor
        # image_curr: <Transform>
        # labels: np.array