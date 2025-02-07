import cv2
import numpy as np

class KalmanTracker:
    def __init__(self):
        # 初始化卡尔曼滤波器
        self.kalman = cv2.KalmanFilter(4, 2)  # 4个状态变量 (x, y, dx, dy), 2个测量值 (x, y)
        
        # 状态转移矩阵 F
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], 
                                                 [0, 1, 0, 1], 
                                                 [0, 0, 1, 0], 
                                                 [0, 0, 0, 1]], dtype=np.float32)
        
        # 观测矩阵 H
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], 
                                                  [0, 1, 0, 0]], dtype=np.float32)
        
        # 过程噪声协方差矩阵 Q
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        
        # 观测噪声协方差矩阵 R
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.05
        
        # 误差协方差矩阵 P
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        
        # 初始状态
        self.kalman.statePost = np.zeros((4, 1), dtype=np.float32)

    def update(self, detection):
        """
        用YOLO的检测结果更新卡尔曼滤波器
        :param detection: (x, y)
        """
        if detection is not None:
            measurement = np.array([[np.float32(detection[0])], [np.float32(detection[1])]])
            self.kalman.correct(measurement)  # 用新观测值更新状态
        prediction = self.kalman.predict()  # 预测下一时刻的位置
        return int(prediction[0]), int(prediction[1])  # 返回预测的位置

