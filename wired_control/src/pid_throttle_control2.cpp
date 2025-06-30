#include "ros/ros.h"
#include "wired_control/PCAN_Data.h"
#include "geometry_msgs/Pose2D.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <ctime>  // 添加头文件
#include <sstream>  // 用于生成文件名字符串

// PID 参数
double Kp = 200.0;
double Ki = 7.0;
double Kd = 0.0;

// PID 状态变量
double integral = 0.0;
double prev_error = 0.0;
double dt = 0.1;  // 控制周期 (单位: 秒)

// 控制目标速度（单位：m/s）
double target_speed = 0.0;

// 发布器（油门控制）
ros::Publisher throttle_pub;

// 日志文件流
std::ofstream log_file;

// 回调函数：接收当前车速并进行 PID 控制
void velocityCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    static double time_elapsed = 0.0;
    time_elapsed += dt;

    double current_speed = msg->theta;  // 当前车速 m/s
    double error = target_speed - current_speed;

    // === 误差死区控制 ===
    if (std::abs(error) < 0.05) {  // 小于 0.05 m/s 认为车速已足够接近目标
        ROS_WARN("进入误差死区，误差 %.3f m/s，保持油门开度不变", error);
        return;
    }

    // PID计算
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;

    // 限幅
    if (output > 1000) output = 1000;
    if (output < 0) output = 0;

     // === 输出死区补偿（起步阶段强制最小油门） ===
    const int DEAD_ZONE_VAL = 150;  // 15% 油门，对应 0~1000 区间
    if (current_speed < 0.1 && output > 0 && output < DEAD_ZONE_VAL) {
        output = DEAD_ZONE_VAL;
        ROS_WARN("起步输出死区补偿：设定油门 %.1f%%", output/10);
    }

    int val = static_cast<int>(output);

    // 构造并发送油门CAN报文
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

    ROS_INFO("目标车速: %.2f km/h, 当前车速: %.2f km/h, 输出油门开度: %d %%", 
              target_speed * 3.6, current_speed * 3.6, val / 10);

    // 写入 CSV 日志
    if (log_file.is_open()) {
        log_file << std::fixed << std::setprecision(2)
                 << time_elapsed << ","
                 << target_speed*3.6 << ","
                 << current_speed*3.6 << ","
                 << val/10 << std::endl;
    }
    prev_error = error;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_speed_control2");
    setlocale(LC_ALL, "");

    if (argc != 2) {
        ROS_ERROR("请提供目标速度，例如: rosrun wired_control pid_speed_control 2.0");
        return 1;
    }

    target_speed = atof(argv[1]);  // 单位 m/s   ***后续可以加上 if else 语句 根据不同的target_speed调节不同的pid参数***

    ros::NodeHandle nh;
    throttle_pub = nh.advertise<wired_control::PCAN_Data>("pcan_data_send", 10);
    ros::Subscriber sub = nh.subscribe("pose_velocity", 10, velocityCallback);

    // === 自动生成文件名：根据 PID 参数 ===
    std::ostringstream filename_stream;
    filename_stream << "pid_log_Kp" << static_cast<int>(Kp)
                    << "_Ki" << static_cast<int>(Ki)
                    << "_Kd" << static_cast<int>(Kd) << ".csv";
    std::string filename = filename_stream.str();
    // 打开CSV日志文件
    log_file.open(filename, std::ios::out);
    if (!log_file) {
        ROS_ERROR("无法打开日志文件 %s", filename.c_str());
        return 1;
    }

    // 写入参数信息和表头
    log_file << "# Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << "\n";
    log_file << "time(s),target_speed(km/h),current_speed(km/h),throttle_val(%)\n";

     ROS_INFO("PID油门控制器已启动，目标车速: %.2f m/s，日志记录为: %s", target_speed, filename.c_str());

    ros::Rate loop_rate(10);  // 控制频率 10Hz
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    log_file.close();
    ROS_INFO("实验完成，日志保存为 %s", filename.c_str());
    return 0;
}
