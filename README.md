# Aelos&Roban选拔任务

## Aelos&Roban选拔任务1.1.1：opencv框选红色小球并输出坐标

#### 1.编写hsv_try.py：可手动调整HSV阈值的GUI界面：

编写1.1/hsv_try.py文件，可通过滑块对一个文件夹中的所有图片进行六个HSV阈值的调整并实时观察效果，通过"w","s","a"按键切换不同图片并保存，保存阈值在hsv_config.txt文件中，下次打开自动更新

![2025-02-06 20-46-35 的屏幕截图](https://github.com/user-attachments/assets/d7845c08-ca40-4a78-9912-0d7ea912c819)


#### 2.编写1.1.py:   opencv框选红色小球并输出坐标

从图片中裁剪特定区域并转换为HSV颜色空间，通过颜色阈值分割提取特定颜色的目标区域，进行形态学处理以优化目标区域的形态，通过轮廓检测找到目标物体并用面积大小筛选，使用红色矩形框和质心标记其位置，绘制坐标，最终显示处理后的结果，并提供可选的图片保存功能。

效果可见： https://github.com/chuchen0210/HITIRC_lyz/issues/1

## Aelos&Roban选拔任务1.1.2：编写发布者订阅者对给定图像进行处理

工作包为IRC/src/opencv，只上传了代码以及CMakeLists.txt与package.xml配置文件

效果可见：https://github.com/chuchen0210/HITIRC_lyz/issues/2

```sh
#终端1
roscore
#终端2
rosrun opencv img_pub.py
#终端3
rosrun opencv img_sub.py
#终端4
rqt_image_view
```

## Aelos&Roban选拔任务1.1.2：编写发布者订阅者对视频流进行处理，识别红色物体（苹果）

```sh
#终端1
roscore
#终端2
rosrun opencv img_pub_v.py
#终端3
rosrun opencv img_sub_v.py
```

效果可见： https://github.com/chuchen0210/HITIRC_lyz/issues/3

## Aelos&Roban选拔任务1.2：跑通yolo，将yolo识别到的物体在视频中的像素坐标通过ros话题发布

跑通yolov8，完成yolo配置，数据集标注，数据集划分（训练集，验证集8：2随机分配）模型训练，模型推理等工作，辅助工具在IRC/src/yolo/tools文件夹，包含图片与视频互相转化，数据集划分等代码，工作包为IRC/src/yolo，只上传了代码以及CMakeLists.txt与package.xml配置文件(weights为kuavo任务模型权重)

```sh
#终端1
roscore
#终端2
conda activate yolov8  #base环境没有torch
python ~/IRC/src/yolo/scripts/yolo_pub.py 
#终端3
rosrun yolo yolo_sub.py
```

使用origincar在赛道上采图并四分类训练，详细工作及效果可见：

https://github.com/chuchen0210/HITIRC_lyz/issues/4

## Aelos&Roban选拔任务1.3：SLAM任务



# Kuavo选拔任务

## Kuavo选拔任务2.1：学习论文，熟悉基于MPC（Model Predictive Control）和WBC（Whole Body Control）机器人控制框架



## Kuavo选拔任务2.2.1：跑通osc2四足机器人，控制其移动到指定位置，明白其中参数的含义

效果可见： https://github.com/chuchen0210/HITIRC_lyz/issues/6

**参数含义：**

-  X：机器人沿地图向前方向运动的位移（与机器人朝向无关，“向前”指地图的方向），绝对值最大为10

-  Y：机器人沿地图向左方向运动的位移（与机器人朝向无关，“向左”指地图的方向），绝对值最大为10
-  Z：机器人沿竖直方向向上运动的位移，绝对值最大为0.2
-  Yaw (deg)：机器人沿自身Z轴逆时针旋转的角度（角度制），绝对值最大为360

## Kuavo选拔任务2.2.2：跑通双足机器人，编写一个脚本来向话题/cmd_vel发送数据控制机器人以指定的速度移动

编写cmd_vel_keyboard_controller.cpp，固定速度大小，在walk状态下，通过按键向话题/cmd_vel发送geometry_msgs::Twist消息控制双足机器人前后左右以及旋转运动

```sh
rosrun humanoid_controllers cmd_vel_keyboard_controller
```

![2025-02-07 17-38-43 的屏幕截图](/home/lyz/图片/2025-02-07 17-38-43 的屏幕截图.png)

效果可见：https://github.com/chuchen0210/HITIRC_lyz/issues/8

## Kuavo选拔任务2.2.3：实现双足机器人自动切换步态的功能

编写gait_change.cpp，控制机器人在stance状态下向/cmd_vel发送不为 0 的速度可以自动切换成walk状态并运动，同时发送为 0 的速度可以自动切换为stance状态并停止

```cpp
        // 根据速度发送步态命令
        if (cmd_vel_msg.linear.x != 0 || cmd_vel_msg.linear.y != 0 || cmd_vel_msg.linear.z != 0 || cmd_vel_msg.angular.z != 0) {
            publishGaitCommand("walk");  // 速度不为零时，发送“walk”命令
        } else {
            publishGaitCommand("stance");  // 速度为零时，发送“stance”命令
        }
```

参考GaitKeyboardPublisher.cpp，通过`rosmsg show ocs2_msgs/mode_schedule`查找ocs2_msgs::mode_schedule消息类型，通过终端信息锁定mode_msg.modeSequence，mode_msg.eventTimes 与步态类型的对应关系

```cpp
// 发布步态命令
void publishGaitCommand(const std::string& gaitCommand) {
    ocs2_msgs::mode_schedule mode_msg;

    // 根据步态命令选择 modeSequence 和 eventTimes
    if (gaitCommand == "walk") {
        // walk 步态的模式序列和时间点
        mode_msg.modeSequence = {1, 3, 2, 3};  // LC, STANCE, RC, STANCE
        mode_msg.eventTimes = {0.0, 0.45, 0.6, 1.05, 1.2};
    } else if (gaitCommand == "stance") {
        // stance 步态的模式序列和时间点
        mode_msg.modeSequence = {3};  // STANCE
        mode_msg.eventTimes = {0.0,1000.0};
    }

    // 发布步态命令
    gaitPublisher_.publish(mode_msg);
}
```

```sh
#放在humanoid_controllers工作包下
rosrun humanoid_controllers gait_change
```

效果可见：https://github.com/chuchen0210/HITIRC_lyz/issues/9

## Kuavo选拨任务3.1：跑通yolo，将yolo识别到的物体在视频中的像素坐标通过ros话题发布

同Aelos&Roban选拔任务1.2，效果可见：https://github.com/chuchen0210/HITIRC_lyz/issues/4

## Kuavo选拨任务3.2：YOLO：使用门把手数据集进行训练并验证

将学长提供的数据集与自己的数据集（手机采图并标注）一起训练，将学长数据集图片转化为视频与手机采的视频合并，进行推理验证。模型权重文件在IRC/srcyolo/weights文件夹下

效果可见：https://github.com/chuchen0210/HITIRC_lyz/issues/5

## Kuavo选拨任务3.3：YOLO：加入卡尔曼滤波，增强跟踪效果

使用IRC/srcyolo/weights/best.pt模型权重，加入卡尔曼滤波效果，当yolo检测对象为空时，卡尔曼滤波的添加可使得yolo短时间内仍然保持跟踪，代码实现在IRC/src/yolo/scripts下的kalman.py与yolo_kf_pub.py中，使用cv2.circle与cv2.putText凸显效果

效果可见：https://github.com/chuchen0210/HITIRC_lyz/issues/7

