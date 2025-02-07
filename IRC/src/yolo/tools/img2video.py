import cv2
import os

def images_to_video(image_folder, output_video, fps=30, resolution=(1920, 1080)):
    # 获取文件夹中的所有图片文件
    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg") or img.endswith(".png")]
    
    # 排序图片文件，以确保按顺序合成视频
    images.sort()

    # 如果文件夹中没有图片，退出函数
    if len(images) == 0:
        print("No images found in the folder.")
        return

    # 获取视频的编码方式（比如 H.264）
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    
    # 创建 VideoWriter 对象，指定输出视频文件、编码、帧率和分辨率
    out = cv2.VideoWriter(output_video, fourcc, fps, resolution)

    # 逐一读取图片并写入视频
    for image in images:
        image_path = os.path.join(image_folder, image)
        frame = cv2.imread(image_path)
        
        # 如果图像的尺寸与目标分辨率不一致，调整图像大小
        if frame.shape[1] != resolution[0] or frame.shape[0] != resolution[1]:
            frame = cv2.resize(frame, resolution)

        out.write(frame)  # 将帧写入视频

    # 释放 VideoWriter 对象
    out.release()
    print(f"Video has been saved to {output_video}")

# 示例用法
image_folder = '/home/lyz/yolov8/train/datasets/handler/images/train'  # 图片文件夹路径
output_video = '/home/lyz/yolov8/output.avi'  # 输出视频文件路径
fps = 20  # 视频帧率
resolution = (1920, 1080)  # 输出视频分辨率

images_to_video(image_folder, output_video, fps, resolution)

