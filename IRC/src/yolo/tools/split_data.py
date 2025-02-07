import os
import random
import shutil

# 定义文件夹路径
images_dir = '/home/lyz/yolov8/train/datasets/handler1/images'
labels_dir = '/home/lyz/yolov8/train/datasets/handler1/labels'
train_images_dir = os.path.join(images_dir, 'train')
val_images_dir = os.path.join(images_dir, 'val')
train_labels_dir = os.path.join(labels_dir, 'train')
val_labels_dir = os.path.join(labels_dir, 'val')

# 创建train和val文件夹（如果它们不存在）
os.makedirs(train_images_dir, exist_ok=True)
os.makedirs(val_images_dir, exist_ok=True)
os.makedirs(train_labels_dir, exist_ok=True)
os.makedirs(val_labels_dir, exist_ok=True)

# 获取images文件夹中的所有图片文件名
image_files = [f for f in os.listdir(images_dir) if f.endswith('.jpg')]

# 随机打乱文件顺序
random.shuffle(image_files)

# 计算训练集和验证集的划分
train_size = int(0.8 * len(image_files))
train_files = image_files[:train_size]
val_files = image_files[train_size:]

# 将图片和标签文件分别复制到对应的train和val文件夹
for image_file in train_files:
    # 图片文件路径
    image_path = os.path.join(images_dir, image_file)
    # 标签文件路径，假设标签文件的格式为 .txt，且文件名与图片文件相同
    label_path = os.path.join(labels_dir, image_file.replace('.jpg', '.txt'))
    
    # 将图片和标签文件复制到train目录
    shutil.copy(image_path, train_images_dir)
    shutil.copy(label_path, train_labels_dir)

for image_file in val_files:
    # 图片文件路径
    image_path = os.path.join(images_dir, image_file)
    # 标签文件路径
    label_path = os.path.join(labels_dir, image_file.replace('.jpg', '.txt'))
    
    # 将图片和标签文件复制到val目录
    shutil.copy(image_path, val_images_dir)
    shutil.copy(label_path, val_labels_dir)

print("划分完成！")

