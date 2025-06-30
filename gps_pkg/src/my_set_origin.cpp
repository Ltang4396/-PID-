//将当前作为坐标原点的车辆在地心地固坐标系的xy值以及所处位置的经纬度值传给坐标转换代码my_gps_conversion
#include <ros/ros.h>                            // 包含ROS头文件
#include <geometry_msgs/Pose2D.h>

class XYZControl
{
public:
    double ref_ecef_x;
    double ref_ecef_y;
    double ref_ecef_z;
    int xy_rev_state;//接收状态
    void XY_Read_Pose(const geometry_msgs::Pose2D &msg);
};

void XYZControl::XY_Read_Pose(const geometry_msgs::Pose2D &msg)
{
    xy_rev_state = 1;
    ref_ecef_x = msg.x;
    ref_ecef_y = msg.y;
    ref_ecef_z = msg.theta;
//    ROS_INFO("x = %f,y = %f",pose_x,pose_y);
}

class LLAControl
{
public:
    double longitude;
    double latitude;
    int LL_rev_state;// 标志位：是否接收到数据（0=未接收，1=已接收）

    void LL_Read_Pose(const geometry_msgs::Pose2D &msg);
};

void LLAControl::LL_Read_Pose(const geometry_msgs::Pose2D &msg)
{
    LL_rev_state = 1;
    longitude = msg.x;
    latitude = msg.y;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"set_gnss_origin");
    ros::NodeHandle nh;
    
    XYZControl xyzcontrol;        // 创建XYControl对象
    LLAControl llacontrol;        // 创建LLControl对象

    int time = 0; // 超时计数器
    // 订阅"pose_xy"话题，回调函数为XYControl::XY_Read_Pose
    ros::Subscriber sub1 = nh.subscribe("pose_xy", 100, &XYZControl::XY_Read_Pose,&xyzcontrol);
    // 订阅"pose_LLA"话题，回调函数为LLControl::LL_Read_Pose
    ros::Subscriber sub2 = nh.subscribe("pose_LLA", 100, &LLAControl::LL_Read_Pose,&llacontrol);
     // 初始化XYControl对象的成员变量
        //是变量的默认值，仅在未接收到话题数据时有效。
     xyzcontrol.ref_ecef_x = 0.0;     // 初始ECEF x值
     xyzcontrol.ref_ecef_y = 0.0;     // 初始ECEF y值
     xyzcontrol.ref_ecef_z = 0.0;     // 初始ECEF z值
     xyzcontrol.xy_rev_state = 0;     // 初始接收状态为未接收
     // 初始化LLControl对象的成员变量
     llacontrol.longitude = 0.0;      // 初始经度值
     llacontrol.latitude = 0.0;       // 初始纬度值
     llacontrol.LL_rev_state = 0;     // 初始接收状态为未接收
    ros::Duration(0.5).sleep();// 等待0.5秒，确保ROS通信初始化完成

    ros::Rate loop_rate(100);// 设置循环频率为100Hz

    //参考坐标原点
     // 等待直到两个数据都接收或超时
     while (ros::ok() && time <= 500) {
        ros::spinOnce(); // 处理回调

        if (xyzcontrol.xy_rev_state && llacontrol.LL_rev_state) {
            break; // 数据已接收，退出循环
        }

        loop_rate.sleep();
        time++;
    }

    if (time > 500) {
        ROS_ERROR("等待数据超时，请检查话题发布！");
        return -1;
    }

    // 设置参数并打印信息
    nh.setParam("ref_ecef_x", xyzcontrol.ref_ecef_x);
    nh.setParam("ref_ecef_y", xyzcontrol.ref_ecef_y);
    nh.setParam("ref_ecef_z", xyzcontrol.ref_ecef_z);

    nh.setParam("longitude", llacontrol.longitude);
    nh.setParam("latitude", llacontrol.latitude);

    ROS_INFO("ECEF参考坐标: x=%.3lf, y=%.3lf, z=%.3lf", xyzcontrol.ref_ecef_x , xyzcontrol.ref_ecef_y , xyzcontrol.ref_ecef_z);
    ROS_WARN("经纬度: 经度=%.6lf, 纬度=%.6lf",llacontrol.longitude, llacontrol.latitude );

    return 0;
}

