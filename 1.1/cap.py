import cv2

# 打开摄像头
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    # 捕获视频帧
    ret, frame = cap.read()

    if not ret:
        print("无法获取帧")
        break

    # 显示视频流
    cv2.imshow('Camera Feed', frame)

    # 按下 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()
