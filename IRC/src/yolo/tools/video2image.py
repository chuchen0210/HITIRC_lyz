import cv2
import os

# 视频文件路径
video_path = '/home/lyz/yolov8/handler.mp4'
# 图片保存文件夹
output_folder = '/home/lyz/yolov8/handler_images'

# 创建保存图片的文件夹（如果不存在）
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 读取视频
cap = cv2.VideoCapture(video_path)

# 获取视频的帧率（FPS）
fps = cap.get(cv2.CAP_PROP_FPS)

# 获取视频的总帧数
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# 循环读取每一帧
frame_count = 0
while True:
    # 读取一帧
    ret, frame = cap.read()

    # 如果没有帧了，退出循环
    if not ret:
        break

    # 保存帧为图片
    frame_filename = os.path.join(output_folder, f'frame_{frame_count:04d}.jpg')
    cv2.imwrite(frame_filename, frame)

    # 输出进度
    print(f'Processing frame {frame_count + 1}/{total_frames}')

    # 增加帧计数
    frame_count += 1

# 释放视频对象
cap.release()

print("Video converted to images successfully.")

