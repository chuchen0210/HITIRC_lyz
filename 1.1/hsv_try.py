import cv2
import numpy as np
import os

# 定义文件路径和文件夹路径
folder_path = r'/home/lyz/1.1/pictures'
config_file = 'hsv_config.txt'

# 读取配置文件中的HSV阈值
def read_hsv_config():
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            values = f.readline().split(',')
            return [int(v) for v in values]
    return [0, 0, 0, 255, 255, 255]  # 默认值

# 保存HSV阈值到配置文件
def save_hsv_config(lower_hsv, upper_hsv):
    with open(config_file, 'w') as f:
        f.write(f'{lower_hsv[0]},{lower_hsv[1]},{lower_hsv[2]},'
                f'{upper_hsv[0]},{upper_hsv[1]},{upper_hsv[2]}')

# 加载文件夹中的图片列表
def load_image_files(folder_path):
    images = [os.path.join(folder_path, f) for f in os.listdir(folder_path)
              if f.endswith(('.jpg', '.jpeg', '.png'))]
    return images

# 读取初始的HSV阈值
hsv_values = read_hsv_config()
l_h, l_s, l_v, u_h, u_s, u_v = hsv_values

# 创建一个窗口和滑块
def nothing(x):
    pass

cv2.namedWindow('Trackbars')

# 创建6个滑块，调整 HSV 阈值
cv2.createTrackbar('LH', 'Trackbars', l_h, 255, nothing)
cv2.createTrackbar('LS', 'Trackbars', l_s, 255, nothing)
cv2.createTrackbar('LV', 'Trackbars', l_v, 255, nothing)
cv2.createTrackbar('UH', 'Trackbars', u_h, 255, nothing)
cv2.createTrackbar('US', 'Trackbars', u_s, 255, nothing)
cv2.createTrackbar('UV', 'Trackbars', u_v, 255, nothing)

# 加载图片列表
image_files = load_image_files(folder_path)
image_index = 0

# 循环处理每一张图片
while True:
    # 读取当前图片
    image = cv2.imread(image_files[image_index])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 获取滑块值
    l_h = cv2.getTrackbarPos('LH', 'Trackbars')
    l_s = cv2.getTrackbarPos('LS', 'Trackbars')
    l_v = cv2.getTrackbarPos('LV', 'Trackbars')
    u_h = cv2.getTrackbarPos('UH', 'Trackbars')
    u_s = cv2.getTrackbarPos('US', 'Trackbars')
    u_v = cv2.getTrackbarPos('UV', 'Trackbars')

    lower_hsv = np.array([l_h, l_s, l_v])
    upper_hsv = np.array([u_h, u_s, u_v])

    # 应用阈值
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # 显示结果
    result = cv2.bitwise_and(image, image, mask=mask)
    cv2.imshow('Result', result)

    # 等待按键输入
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif key == ord('s'):  # 按's'键切换到下一张图片
        image_index = (image_index + 1) % len(image_files)
    elif key == ord('w'):  # 按'w'键切换到上一张图片
        image_index = (image_index - 1) % len(image_files)
    elif key == ord('a'):  # 按'a'键保存当前HSV阈值
        save_hsv_config(lower_hsv, upper_hsv)
        print('HSV阈值已保存！')

cv2.destroyAllWindows()
