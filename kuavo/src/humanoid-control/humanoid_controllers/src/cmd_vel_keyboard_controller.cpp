#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// 固定的线速度和角速度
const double TARGET_LINEAR_VELOCITY = 0.5;  // 线速度 (m/s)
const double TARGET_ANGULAR_VELOCITY = 0.5; // 角速度 (rad/s)

class CmdVelKeyboardController {
public:
    CmdVelKeyboardController() {
        ros::NodeHandle nh;
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        ROS_INFO("Fixed Speed: Linear=%.2f m/s, Angular=%.2f rad/s", TARGET_LINEAR_VELOCITY, TARGET_ANGULAR_VELOCITY);
        printInstructions();
    }

    // 运行键盘控制循环
    void run() {
        char key;
        while (ros::ok()) {
            key = getKeyPress();

            geometry_msgs::Twist cmd_vel_msg;
            switch (key) {
                case 'w':  // 前进
                    cmd_vel_msg.linear.x = TARGET_LINEAR_VELOCITY;
                    break;
                case 's':  // 后退
                    cmd_vel_msg.linear.x = -TARGET_LINEAR_VELOCITY;
                    break;
                case 'a':  // 左移
                    cmd_vel_msg.linear.y = TARGET_LINEAR_VELOCITY;
                    break;
                case 'd':  // 右移
                    cmd_vel_msg.linear.y = -TARGET_LINEAR_VELOCITY;
                    break;
                case 'q':  // 上升
                    cmd_vel_msg.linear.z = TARGET_LINEAR_VELOCITY;
                    break;
                case 'e':  // 下降
                    cmd_vel_msg.linear.z = -TARGET_LINEAR_VELOCITY;
                    break;
                case 'z':  // 逆时针旋转
                    cmd_vel_msg.angular.z = TARGET_ANGULAR_VELOCITY;
                    break;
                case 'c':  // 顺时针旋转
                    cmd_vel_msg.angular.z = -TARGET_ANGULAR_VELOCITY;
                    break;
                case 'x':  // 停止机器人
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.linear.y = 0;
                    cmd_vel_msg.linear.z = 0;
                    cmd_vel_msg.angular.z = 0;
                    break;
                case 'h':  // 显示帮助
                    printInstructions();
                    continue;
                default:
                    continue;
            }

            // 发送速度命令
            cmd_vel_pub_.publish(cmd_vel_msg);
            ROS_INFO("Sent Command: Linear(%.2f, %.2f, %.2f) | Angular(%.2f)", 
                     cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.linear.z, cmd_vel_msg.angular.z);

            ros::spinOnce();
        }
    }

private:
    ros::Publisher cmd_vel_pub_;

    // 监听键盘输入
    char getKeyPress() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    // 显示帮助信息
    void printInstructions() {
        std::cout << "\n===== Robot Keyboard Control =====\n"
                  << "Use the following keys to control the robot:\n"
                  << "  W  - Move Forward\n"
                  << "  S  - Move Backward\n"
                  << "  A  - Move Left\n"
                  << "  D  - Move Right\n"
                  << "  Q  - Move Up\n"
                  << "  E  - Move Down\n"
                  << "  Z  - Rotate Counterclockwise\n"
                  << "  C  - Rotate Clockwise\n"
                  << "  X  - Stop\n"
                  << "  H  - Show this help message\n"
                  << "Press Ctrl+C to exit\n"
                  << "==================================\n";
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_keyboard_controller");
    CmdVelKeyboardController controller;
    controller.run();
    return 0;
}

