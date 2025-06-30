//在第二版的基础上新增了考虑z的y轴真实值
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <cmath>

class XYZControl
{
public:
    double ref_ecef_x;
    double ref_ecef_y;
    double ref_ecef_z;
    void XYZ_Read_Pose(const geometry_msgs::Pose2D &msg);
};

void XYZControl::XYZ_Read_Pose(const geometry_msgs::Pose2D &msg)
{
    ref_ecef_x = msg.x;
    ref_ecef_y = msg.y;
    ref_ecef_z = msg.theta;
}

class LLAControl
{
public:
    double longitude;
    double latitude;
    void LLA_Read_Pose(const geometry_msgs::Pose2D &msg);
};

void LLAControl::LLA_Read_Pose(const geometry_msgs::Pose2D &msg)
{
    longitude = msg.x;
    latitude = msg.y;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"gnss_enu_convert");
    ros::NodeHandle nh;
    XYZControl xyzcontrol;
    LLAControl llacontrol;
    ros::Subscriber sub3 = nh.subscribe("pose_xy", 10, &XYZControl::XYZ_Read_Pose,&xyzcontrol);   //from 100 to 20
    ros::Subscriber sub4 = nh.subscribe("pose_LLA",10,&LLAControl::LLA_Read_Pose,&llacontrol);
    ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>("gnss_relative", 10);  // 创建GPS数据发布器,from 1000 to 10
    ros::Rate loop_rate(10);
    geometry_msgs::Pose2D pose;  
    double x_relat = 0.0;
    double y_relat = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double ref_ecef_x = 0.0;
    double ref_ecef_y = 0.0;
    double ref_ecef_z = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    //获取坐标原点
    nh.getParam("ref_ecef_x",ref_ecef_x);
    nh.getParam("ref_ecef_y",ref_ecef_y);
    nh.getParam("ref_ecef_z",ref_ecef_z);
    nh.getParam("longitude",longitude);
    nh.getParam("latitude",latitude);
    const double lon_rad = longitude * M_PI / 180.0;
    const double lat_rad = latitude * M_PI / 180.0;
    const double sin_lon = sin(lon_rad);
    const double cos_lon = cos(lon_rad);
    const double sin_lat = sin(lat_rad);
    const double cos_lat = cos(lat_rad);
    ROS_WARN("收到的ref_ecef_x=%.3f ref_ecef_y=%.3f ref_ecef_z=%.3f",ref_ecef_x,ref_ecef_y,ref_ecef_z);
    ROS_WARN("收到的longitude=%.3f latitude=%.3f",longitude,latitude);
    //参考坐标原点
    while(ros::ok())
    {
        //读取实时GNSS原始数据
        x = xyzcontrol.ref_ecef_x;
        y = xyzcontrol.ref_ecef_y;
        z = xyzcontrol.ref_ecef_z;
        //坐标转化
        x = (x-ref_ecef_x);
        y = (y-ref_ecef_y); 
        z = (z-ref_ecef_z); 
        x_relat = -sin_lon * x + cos_lon * y;  // sin 和 cos 默认是弧度制的
        y_relat = -sin_lat * cos_lon * x - sin_lat * sin_lon * y + cos_lat * z;
        pose.x = x_relat;
        pose.y = y_relat;
        ROS_INFO("当前x位置为 %.3f m, 当前y位置为 %.3f m", x_relat, y_relat);
        pub.publish(pose);  // 发布GPS数据到ROS话题
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
