import numpy as np
import cv2

# 加载指定的图片
image_path = r'E:\IRC\pictures\1.jpg'
frame = cv2.imread(image_path)

# 检查图片是否成功加载
if frame is None:
    print("无法加载图片")
else:
    # 裁剪图片并转换为HSV颜色空间
    top=320
    im0 = frame[top:640, 0:480]  #0上下，1左右
    hsv_img = cv2.cvtColor(im0, cv2.COLOR_BGR2HSV)

    # 定义目标颜色的HSV范围
    lower_hsv = np.array([16, 0, 0])
    upper_hsv = np.array([255, 255, 255])


    # 根据颜色范围生成二值化图像（阈值化处理）
    imgThresh = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

    cv2.imshow('Threshed Image', imgThresh)  # 显示二值化图像

    # 噪声去除
    kernel = np.ones((5, 5), np.uint8)
    smoothed_img = cv2.dilate(imgThresh, kernel, iterations=11)
    smoothed_img = cv2.erode(smoothed_img, kernel, iterations=7)

    # 查找轮廓
    contours, hierarchy = cv2.findContours(255 - imgThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 初始化变量
    max_area = 0
    largest_rect = None
    largest_cnt = None

    # 遍历轮廓
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1200:
            x, y, w, h = cv2.boundingRect(cnt)
            if area > max_area:
                max_area = area
                largest_rect = (x, y, w, h)
                largest_cnt = cnt

    # 绘制轮廓和矩形框
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1200:
            x, y, w, h = cv2.boundingRect(cnt)

            # 绘制矩形框
            if (x, y, w, h) == largest_rect:
                color = (0, 0, 255)  # 红色
                cv2.rectangle(frame, (x , y + top ), (x + w , y + h + top), color, 3)
                cv2.circle(frame, (x , y + top +h), 5, (255, 0, 0), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                lower_left_text = f"({x}, {y+top+h})"
                cv2.putText(frame, lower_left_text, (x-60, y + top + h + 30), font, 0.6, (0, 0, 255), 2)
            else:
                color = (0, 255, 0)  # 绿色
                # cv2.rectangle(frame, (x + 50, y + 120), (x + w + 50, y + h + 120), color, 3)

            # 绘制质心
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"]) + top
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                #cv2.putText(frame, 'Cone Center', (cx, cy - 10), font, 0.6, (0, 0, 255), 2)
                center_text = f"({cx}, {cy})"
                cv2.putText(frame, center_text, (cx + 40, cy+10), font, 0.6, (0, 0, 255), 2)

    # 显示处理后的图像
    cv2.imshow('Processed Image', frame)
    # 保存
    output_path = r'E:\IRC\pictures\out_1.jpg'
    cv2.imwrite(output_path, frame)
    # 按任意键退出
    cv2.waitKey(0)

# 释放所有窗口
cv2.destroyAllWindows()
