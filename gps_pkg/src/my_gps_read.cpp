// 在原来的基础上增加了打印功能，替换了原来复杂难度的
//同时将其输出在detail_output.txt文件夹中

#include <ros/ros.h>                            // 包含ROS头文件
#include <serial/serial.h>                      // 包含串口通信库头文件
#include <std_msgs/String.h>                    // 包含ROS标准消息类型头文件
#include <std_msgs/Empty.h>                     // 包含ROS标准消息类型头文件>
#include <geometry_msgs/Pose2D.h>  // x y theta
//写入txt文件需要加的头文件
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;                            // 使用标准命名空间

template <class Type>
Type stringToNum(const string str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gnss_read");     // 初始化ROS节点
    ros::NodeHandle nh; // 创建ROS节点句柄, 
    std::ofstream file("detail_output.txt", std::ios::app); // 追加模式
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Failed to open output file!");
        return -1;
    }
    ros::Publisher gnss_pub = nh.advertise<geometry_msgs::Pose2D>("pose_xy", 10);  // 创建地心地固坐标系的x和y
    ros::Publisher pose_angle_pub = nh.advertise<geometry_msgs::Pose2D>("pose_angle", 10);  // 创建GPS数据发布器 from 1000 to 10
    ros::Publisher pose_gyro_pub = nh.advertise<geometry_msgs::Pose2D>("pose_gyro", 1000);  // 创建GPS数据发布器
    ros::Publisher pose_acc_pub = nh.advertise<geometry_msgs::Pose2D>("pose_acc", 1000);  // 创建GPS数据发布器
    ros::Publisher pose_velocity_pub = nh.advertise<geometry_msgs::Pose2D>("pose_velocity", 1000);  // 创建GPS数据发布器
    ros::Publisher pose_LLA_pub = nh.advertise<geometry_msgs::Pose2D>("pose_LLA", 10);  // 创建GPS数据发布器

    geometry_msgs::Pose2D gnss_xy;                   // 创建位移数据消息对象
    geometry_msgs::Pose2D pose_angle;                // 创建角度数据消息对象
    geometry_msgs::Pose2D pose_gyro;                 // 创建角速度数据消息对象
    geometry_msgs::Pose2D pose_acc;                  // 创建加速度数据消息对象
    geometry_msgs::Pose2D pose_velocity;             // 创建速度数据消息对象
    geometry_msgs::Pose2D pose_LLA;                  // 创建经纬海拔数据消息对象

    serial::Serial ser;                              // 创建串口通信对象

    try 
    {
        ser.setPort("/dev/ttyUSB0");                // 设置串口设备路径
        ser.setBaudrate(230400);                    // 设置波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // 设置超时
        ser.setTimeout(to);                         // 应用超时设置
        ser.open();                                 // 打开串口
    } 
    catch (serial::IOException& e) 
    {
        ROS_ERROR_STREAM("Unable to open port ");    // 打开串口失败时输出错误信息
        return -1;                                   // 返回-1表示出错
    }

    if (ser.isOpen())                             // 检查串口是否成功打开
    {
        ROS_INFO_STREAM("Serial Port initialized, Start Recv GPS Data.");  // 打开成功，输出信息
    } 
    else 
    {
        return -1;                                   // 返回-1表示出错
    }
    
    ros::Rate loop_rate(10);                      // 设置ROS循环频率为100Hz,from 100 to 50
    std::string strRece;                           // 创建字符串用于存储串口接收的数据

    while (ros::ok())                             // 主循环，直到ROS节点关闭
    {
        if (ser.available())                        // 检查串口是否有可用数据
        {
            strRece += ser.read(ser.available());     // 读取所有可用数据到字符串中
            std::string gstart = "$G";                // 定义GPS数据的起始标志
            std::string gend = "\r\n";                // 定义GPS数据的结束标志  回车符、换行符
            long i = 0, start = -1, end = -1;

            while (i < strRece.length()) 
            {
                start = strRece.find(gstart);           // 查找GPS数据的起始位置
                if (start == -1)                        // 未找到起始标志
                {
                    if (strRece.length() > 2) 
                    {
                        strRece = strRece.substr(strRece.length() - 3);  // 如果未找到起始标志，保留末尾字符
                    }
                    break;
                } 
                else                                    // 找到起始标志
                {
                    end = strRece.find(gend);             // 查找GPS数据的结束位置
                    if (end == -1) 
                    {
                        if (end != 0) 
                        {
                            strRece = strRece.substr(start);   // 如果未找到结束标志，保留从起始位置开始的字符
                        }
                        break;
                    } 
                    else                                   // 找到结束标志
                    {
                        i = end;
                        std::string frames = strRece.substr(start, end + 2 - start);  // 提取完整的GPS数据帧（起始位置，数据长度）
                        // $GPXYZ,030212.50,-1714918.557,4992705.152,3568149.091*6C
                        if (frames.find("$GPXYZ") != std::string::npos)   // 查找$GNRMC数据类型
                        {
                            //std::cout << frames;
                            vector<string> XYZ_vector;  //定义一个字符串容器
                            int position = 0;
                            // 查找','
                            do {
                                string tmp_s;
                                position = frames.find(",");
                                tmp_s = frames.substr(0, position);
                                frames.erase(0, position + 1);
                                XYZ_vector.push_back(tmp_s);
                            } while (position != -1);  

                            double x = stringToNum<double>(XYZ_vector[2]);
                            double y = stringToNum<double>(XYZ_vector[3]);
                            double z = stringToNum<double>(XYZ_vector[4]);
                            gnss_xy.x = x;
                            gnss_xy.y = y;
                            gnss_xy.theta=z;
                            gnss_pub.publish(gnss_xy);  // 发布GPS数据到ROS话题
                            ROS_INFO_STREAM("当前坐标为: " << x << " , " << y << " , " << z);
                            file << "当前时间: " << ros::Time::now() << "\t坐标: " << x << ", " << y << " , " << z <<std::endl;
                        } 
                        else if (frames.find("$GPCHC") != std::string::npos) 
                        {
                            vector<string> CHC_vector;  //定义一个字符串容器
                            int position = 0;
                            do
                            {
                                string tmp_s;
                                position = frames.find(",");
                                tmp_s = frames.substr(0, position);
                                frames.erase(0, position + 1);
                                CHC_vector.push_back(tmp_s);
                            } while (position != -1); //按照“ ， ”的位置，依次把逗号之间的数据提取到XYZ_vector 方便后续调用
/*
[
    $GPCHC=$GPCHC,2279,184215.60,93.41,-0.34,-0.04,0.01,0.16,-0.06,0.0001,-0.0090,1.0040,34.23400481,108.95683762,384.81,-0.001,0.004,-0.000,0.005,21,21,10,0,2*5E
    "$GPCHC=$GPCHC",  // 帧头      GPCHC协议头
    "2279",           // 字段1    自 1980-1-6 至当前的星期数（GPS 时间）
    "184215.60",      // 字段2      自本周日 0:00:00 至当前的秒数（GPS 时间）
    "93.41",          // 字段3     横摆角  (0-359.99)
    "-0.34",          // 字段4     俯仰角   (-90 至 90)
    "-0.04",          // 字段5      侧倾角  (-180 至180 )
    "0.01",           // 字段6      俯仰角速度  （）
    "0.16",           // 字段7      侧倾角速度
    "-0.06",          // 字段8      横摆角速度
    "0.0001",         // 字段9      x轴加速度   车辆侧向方向
    "-0.0090",        // 字段10     y轴加速度   车辆前进方向
    "1.0040",         // 字段11     z轴加速度   车辆头顶方向
    "34.23400481",    // 字段12     纬度（ -90° 至 90° ）
    "108.95683762",   // 字段13     经度（-180° 至 180°）
    "384.81",         // 字段14     高度，单位（米）
    "-0.001",         // 字段15     东向速度，单位（米/秒）
    "0.004",          // 字段16     北向速度，单位（米/秒）
    "-0.000",         // 字段17     天向速度，单位（米/秒）
    "0.005",          // 字段18     车辆速度，单位（米/秒）
    "21",             // 字段19     主天线 1 卫星数
    "21",             // 字段20     副天线 2 卫星数
    "10",             // 字段21
    "0",              // 字段22     差分延时
    "2*5E"            // 校验和
  ]
*/
                            double heading = stringToNum<double>(CHC_vector[3]);//横摆角，正北为正
                            double pitch = stringToNum<double>(CHC_vector[4]);//俯仰
                            double roll = stringToNum<double>(CHC_vector[5]);//侧倾
                            double gyro_x = stringToNum<double>(CHC_vector[6]);//俯仰角速度
                            double gyro_y = stringToNum<double>(CHC_vector[7]);//侧倾角速度
                            double gyro_z = stringToNum<double>(CHC_vector[8]);//横摆角速度                            
                            double acc_x = stringToNum<double>(CHC_vector[9]);//x轴加速度                       
                            double acc_y = stringToNum<double>(CHC_vector[10]);//y轴加速度                           
                            double acc_z = stringToNum<double>(CHC_vector[11]);//z轴加速度                            
                            double latitude = stringToNum<double>(CHC_vector[12]);//纬度                           
                            double longitude = stringToNum<double>(CHC_vector[13]);//经度                            
                            double Altitude = stringToNum<double>(CHC_vector[14]);//海拔高度
                            double ve = stringToNum<double>(CHC_vector[15]);//东向速度
                            double vn = stringToNum<double>(CHC_vector[16]);//北向速度
                            // double vu = stringToNum<double>(CHC_vector[17]);//天向速度
                            double v = stringToNum<double>(CHC_vector[18]);//车辆速度

                            pose_angle.x=heading; 
                            pose_angle.y=pitch;  
                            pose_angle.theta=roll;   
                            pose_angle_pub.publish(pose_angle);  // 发布角度数据到ROS话题
                            pose_gyro.x=gyro_x; 
                            pose_gyro.y=gyro_y; 
                            pose_gyro.theta=gyro_z;   
                            pose_gyro_pub.publish(pose_gyro);  // 发布角速度数据到ROS话题
                            pose_acc.x=acc_x; 
                            pose_acc.y=acc_y;  
                            pose_acc.theta=acc_z;   
                            pose_acc_pub.publish(pose_acc);  // 发布加速度数据到ROS话题
                            pose_velocity.x=ve; 
                            pose_velocity.y=vn;  
                            pose_velocity.theta=v;   
                            pose_velocity_pub.publish(pose_velocity);  // 发布速度数据到ROS话题
                            pose_LLA.x=longitude;//经度 -180至180
                            pose_LLA.y=latitude; //纬度 -90至90
                            pose_LLA.theta=Altitude; //海拔高度 m
                            pose_LLA_pub.publish(pose_LLA);
                            //没有扒下来经纬度和海拔高度
                            ROS_WARN("\n\t当前横摆角为：%.2f°\n\t当前俯仰角为：%.2f°\n\t当前侧倾角为：%.2f°\n\t"
                                "当前俯仰角速度: %.4f rad/s\n\t当前侧倾角速度: %.4f rad/s\n\t当前横摆角速度: %.4f rad/s\n\t"
                                "当前x轴加速度: %.4f m/s²\n\t当前y轴加速度: %.4f m/s²\n\t当前z轴加速度: %.4f m/s²\n\t"
                                "当前纬度: %.4f °\n\t当前经度: %.4f °\n\t当前海拔高度: %.4f °\n\t"
                                "当前东向速度: %.4f m/s\n\t当前北向速度: %.4f m/s\n\t当前车辆速度: %.2f m/s",
                                heading, pitch, roll, gyro_x, gyro_y, gyro_z, 
                                acc_x, acc_y, acc_z,latitude,longitude,Altitude,ve,vn,v);
                       
                       // 文件输出
                              file <<"当前时间: " << ros::Time::now() 
                                   << "\t\n横摆角: " << heading
                                   << "\t\n俯仰角: " << pitch
                                   << "\t\n侧倾角: " << roll
                                   << "\t\n俯仰角速度: " << gyro_x
                                   << "\t\n侧倾角速度: " << gyro_y
                                   << "\t\n横摆角速度: " << gyro_z
                                   << "\t\nx轴加速度: " << acc_x
                                   << "\t\ny轴加速度: " << acc_y
                                   << "\t\nz轴加速度: " << acc_z
                                   << "\t\n纬度: " << latitude
                                   << "\t\n经度: " << longitude
                                   << "\t\n海拔高度: " << Altitude
                                   << "\t\n东向速度: " << ve
                                   << "\t\n北向速度: " << vn
                                   << "\t\n车速: " << v 
                                   << std::endl;
                                file.flush(); // 确保及时写入
                        } 
                        strRece = strRece.substr(end + 2);  // 从索引end + 2开始提取到字符串末尾的子字符串,移除已处理的数据
                    }
                }
            }
        }
        ros::spinOnce();             // 处理ROS回调函数
        loop_rate.sleep();           // 控制循环频率
    }
    file.close();
    return 0;                      // 返回0表示正常结束程序
}

