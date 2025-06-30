#include "ros/ros.h"
#include "wired_control/PCAN_Data.h"
#include "geometry_msgs/Pose2D.h"
#include <cstdlib>  // for atof
#include <fstream>  // for file IO
#include <iomanip>  // for std::setprecision

// PID 参数
double Kp = 200.0;
double Ki = 7.0;
double Kd = 40;

// PID 状态
double integral = 0.0;
double prev_error = 0.0;
double dt = 0.1;  // 控制周期

// 全局变量
double target_speed = 0.0;  // 将在 main 中从命令行读取
ros::Publisher throttle_pub;  //因为要在订阅的回调函数中使用，所以是全局变量

// 日志文件流
std::ofstream log_file;

void velocityCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    double current_speed = msg->theta;  // 当前车速（单位：m/s）

    double error = target_speed - current_speed;

    // PID计算
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;

    // 限制输出范围（0 ~ 1000）
    if (output > 1000) output = 1000;
    if (output < 0) output = 0;

    int val = static_cast<int>(output);

    // 构造油门控制 CAN 报文
    wired_control::PCAN_Data throttle_msg;
    throttle_msg.id = static_cast<int>(0x1ACC8004);
    throttle_msg.datas[0] = 0x84;
    throttle_msg.datas[1] = 0x02;
    throttle_msg.datas[2] = (val >> 8) & 0xFF;
    throttle_msg.datas[3] = val & 0xFF;
    throttle_msg.datas[4] = 0x00;
    throttle_msg.datas[5] = 0x00;
    throttle_msg.datas[6] = 0x00;
    throttle_msg.datas[7] = 0x00;
    throttle_msg.dlc = 4;
    throttle_msg.msgType = true;

    throttle_pub.publish(throttle_msg);

    ROS_INFO("目标车速: %.2f km/h, 当前车速: %.2f km/h, 输出油门开度: %d %", target_speed*3.6, current_speed*3.6, val/10);
    // 写入日志文件（保存格式同ROS_INFO）
    if (log_file.is_open()) {
        log_file << std::fixed << std::setprecision(2)
                 << "目标车速: " << target_speed << " m/s, "
                 << "当前车速: " << current_speed << " m/s, "
                 << "输出油门开度: " << val/10 << std::endl;
    }
    prev_error = error;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_throttle_control");
    setlocale(LC_ALL,"");//设置中文
    // 检查是否提供了目标车速参数
    if (argc != 2) {
        ROS_ERROR("请提供目标速度，例如: rosrun wired_control pid_throttle_control 10");
        return 1;
    }  // 运行代码为 rosrun wired_control pid_throttle_control

    target_speed = atof(argv[1]);  // 将字符串转为浮点数

    ros::NodeHandle nh;
    throttle_pub = nh.advertise<wired_control::PCAN_Data>("pcan_data_send", 10);
    ros::Subscriber sub = nh.subscribe("pose_velocity", 10, velocityCallback);

    // 打开日志文件，追加写模式
    log_file.open("throttle_output_log.txt", std::ios::out | std::ios::app);
    if (!log_file) {
        ROS_ERROR("无法打开日志文件 throttle_output_log.txt");
        return 1;
    }
    
    ROS_INFO("PID 油门控制器已启动，目标车速: %.2f m/s", target_speed);

    ros::Rate loop_rate(10);  // 10Hz 控制频率
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
