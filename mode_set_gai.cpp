#include "ros/ros.h"                  // 引入ROS核心库，用于创建节点、发布和订阅消息
#include "std_msgs/String.h"          // 引入标准字符串消息类型
#include <std_msgs/Empty.h>           // 引入空消息类型
#include <stdio.h>                    // 引入标准输入输出库
#include "wired_control/PCAN_Data.h"       // 引入自定义的CAN消息类型
#include "wired_control/pcandevice.h"               // 引入PCAN设备操作相关的头文件

// CAN ID
#define steer_ID 0x1ACC8001
#define brake_ID 0x1ACC8002
#define gear_ID  0x1ACC8003
#define throttle_ID 0x1ACC8004
#define light_ID 0x1ACC8005
// 模式帧,默认手动模式
unsigned char steer_mode[8]={0x81,0x01,0x80}; // 转向模式帧数据，默认手动模式 对应excel第一行 81 01 01（80）
unsigned char brake_mode[8]={0x81,0x01,0x80};
unsigned char gear_mode[8]={0x81,0x01,0x80};
unsigned char throttle_mode[8]={0x81,0x01,0x80};
unsigned char light_mode[8]={0x81,0x01,0x80};
// 线控状态标志位
unsigned char steer_flag = 0; // 转向模式状态标志位，0表示未设置，1表示已设置
unsigned char brake_flag = 0;
unsigned char gear_flag = 0;
unsigned char throttle_flag = 0;
unsigned char light_flag = 0;
// 线控状态检查
void wireStateCheck(const wired_control::PCAN_Data &msg)
{
    int id = msg.id;
    unsigned int datas[8] = {0};

    // 将消息的数据部分转换为unsigned int类型
    for (int i = 0;i < 8;i++)
    {
        //强制类型转换运算符 <要转换到的类型> (待转换的表达式)
        datas[i] = static_cast<unsigned int>(msg.datas[i]);
    }
    if(id == steer_ID)// 遥测数据-转向
    {
        if((!steer_flag)&&(datas[0] == 0x40) && (datas[1] == 0x04))
        {
            steer_flag = 1;
            if(datas[2] == 0x01)
            {
                ROS_INFO("Steering mode: wire.");
            }
            else
            {
                ROS_INFO("Steering mode: manual.");
            }
            //提取低字节：datas[5] & 0x00FF

            //提取高字节：(datas[4] << 8) & 0xFF00

            //合并高低字节：(datas[5] & 0x00FF) | ((datas[4] << 8) & 0xFF00)

            //计算转向角度：减去 3600，得到实际的转向角度。
            int steerValue = ((datas[5] & 0x00FF) | ((datas[4] << 8) & 0xFF00)) - 3600;
            ROS_INFO("Steering angle: %d deg",steerValue);
        }
    }
    else if(id == brake_ID)// 遥测数据-刹车
    {
        if((!brake_flag)&&(datas[0] == 0x40) && (datas[1] == 0x04))
        {
            brake_flag = 1;
            if(datas[2] == 0x01)
            {
                ROS_INFO("Brake mode: wire.");
            } else
            {
                ROS_INFO("Brake mode: manual.");
            }
            int brakeValue = (datas[5] & 0x00FF) | ((datas[4] << 8) & 0xFF00);
            ROS_INFO("Brake depth: %d",brakeValue);
        }
    }
    else if(id == gear_ID)// 遥测数据-档位
    {
        if((!gear_flag)&&(datas[0] == 0x40) && (datas[1] == 0x03))
        {
            gear_flag = 1;
            if(datas[2] == 0x01)
            {
                ROS_INFO("Gear mode: wire.");
            }
            else
            {
                ROS_INFO("Gear mode: manual.");
            }
            if (datas[4]==0x81)
            {
               ROS_INFO("Gear position:D gear");
            }
            else if(datas[4]==0x88)
            {
               ROS_INFO("Gear position:R gear");
            }
            else
            {
               ROS_INFO("Gear position:N gear");
            }
        }
    }
    else if(id == throttle_ID)// 遥测数据-油门
    {

        if((!throttle_flag) &&(datas[0] == 0x40) && (datas[1] == 0x04))
        {
            throttle_flag = 1;
            if(datas[2] == 0x01) {
                ROS_INFO("Throttle mode: wire.");
            }
            else
            {
                ROS_INFO("Throttle mode: manual.");
            }
            int throttleValue= (datas[5] & 0x00FF) | ((datas[4] << 8) & 0xFF00);
            ROS_INFO("Throttle depth: %d",(throttleValue/10));
        }
    }
    else if(id == light_ID)// 遥测数据-灯光
    {
        if((!light_flag) && (datas[0] == 0x40) && (datas[1] == 0x03))
        {
            light_flag = 1;
            if(datas[2] == 0x01)
            {
                ROS_INFO("Light mode: wire.");
            }
            else
            {
                ROS_INFO("Light mode: manual.");
            }
        }
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");//设置中文
     // 获取命令中的参数
     if (argc == 1)  // 如果没有传入参数
     {
         // 默认将所有模式设置为线控模式
         steer_mode[2] = 0x01;
         brake_mode[2] = 0x01;
         gear_mode[2] = 0x01;
         throttle_mode[2] = 0x01;
         light_mode[2] = 0x01;
     }
    else if(argc == 6)// 如果传入了5个参数
    {
        // 根据参数部分线控模式
        if(atoi(argv[1])==1)steer_mode[2]=0x01; //此处体现mode_set 1 0 0 0 0 
        if(atoi(argv[2])==1)brake_mode[2]=0x01;
        if(atoi(argv[3])==1)gear_mode[2]=0x01;
        if(atoi(argv[4])==1)throttle_mode[2]=0x01;
        if(atoi(argv[5])==1)light_mode[2]=0x01;
    }
    else  // 如果参数数量不正确
    {
        ROS_INFO("usage:rosrun pcan_usb mode_set [steer] [brake] [gear] [throttle] [light]\nparam:\n[steer   ]: 0:manual;1:wire\n[brake   ]: 0:manual;1:wire\n[gear    ]: 0:manual;1:wire\n[throttle]: 0:manual;1:wire\n[light   ]: 0:manual;1:wire\n");
        return -1;// 退出程序
    }

    ros::init(argc, argv, "mode_set_gai");  // 初始化ROS节点
    ros::NodeHandle nh; // 创建节点句柄
    // 创建发布者，发布CAN消息
    ros::Publisher pub = nh.advertise<wired_control::PCAN_Data>("pcan_data_send", 100);
    wired_control::PCAN_Data pcan_Data;
    ros::Duration(1.0).sleep();//防止节点启动未完成，消息丢失

    // 设置转向模式
    pcan_Data.id = static_cast<int>(steer_ID);
    for (int i = 0; i < 8; i++)
    {
        pcan_Data.datas[i]=steer_mode[i];
    }
    pcan_Data.dlc = 3;  // 设置数据长度
    pcan_Data.msgType = true; // 设置消息类型
    pub.publish(pcan_Data); // 发布消息
    // 转向模式初始化，让方向盘居中
    pcan_Data.datas[0] = 0x84;  // 功能码：设置转向 
    pcan_Data.datas[1] = 0x02;  // 数据长度
    pcan_Data.datas[2] = 0x00;  // 转向高字节
    pcan_Data.datas[3] = 0x00;  // 转向低字节
    pcan_Data.dlc = 4;
    pcan_Data.msgType = true;
    pub.publish(pcan_Data);
    ROS_INFO("方向盘恢复初始设置");
    // 设置刹车模式
    pcan_Data.id = static_cast<int>(brake_ID);
    for (int i = 0; i < 8; i++)
    {
        pcan_Data.datas[i]=brake_mode[i];
    }
    pcan_Data.dlc = 3;  // 设置数据长度
    pcan_Data.msgType = true;
    pub.publish(pcan_Data);
    // 设置挡位模式
    pcan_Data.id = static_cast<int>(gear_ID);
    for (int i = 0; i < 8; i++)
    {
        pcan_Data.datas[i]=gear_mode[i];
    }
    pcan_Data.dlc = 3;
    pcan_Data.msgType = true;
    pub.publish(pcan_Data);

    // 设置油门模式
    pcan_Data.id = static_cast<int>(throttle_ID);
    for (int i = 0; i < 8; i++)
    {
        pcan_Data.datas[i]=throttle_mode[i];
    }
    pcan_Data.dlc = 3;
    pcan_Data.msgType = true;
    pub.publish(pcan_Data);
    // 油门模式初始化
    pcan_Data.id = static_cast<int>(throttle_ID);
    pcan_Data.datas[0] = 0x84;  // 功能码：设置油门
    pcan_Data.datas[1] = 0x02;  // 数据长度
    pcan_Data.datas[2] = 0x00;  // 油门高字节
    pcan_Data.datas[3] = 0x00;  // 油门低字节
    pcan_Data.dlc = 4;
    pcan_Data.msgType = true;
    pub.publish(pcan_Data);
    ROS_INFO("油门开度为 0%% ");
    
    
    // 设置灯光模式
    pcan_Data.id = static_cast<int>(light_ID);
    for (int i = 0; i < 8; i++)
    {
        pcan_Data.datas[i]=light_mode[i];
    }
    pcan_Data.dlc = 3;
    pcan_Data.msgType = true;
    pub.publish(pcan_Data);
    //检查开启线控底盘状态检查
    ros::Subscriber sub = nh.subscribe("pcan_data_recv", 100, &wireStateCheck);

    while(ros::ok())
    {
         // 如果所有模式都已设置，则退出程序
        if(steer_flag && brake_flag && gear_flag && throttle_flag && light_flag)
        {
            ros::shutdown();// 关闭ROS节点
            break;
        }
        ros::spinOnce();// 处理回调函数
    }
    return 0;
}

