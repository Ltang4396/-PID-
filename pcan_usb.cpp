#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <stdio.h>
#include "wired_control/PCAN_Data.h"
#include "wired_control/pcandevice.h"
#include <boost/bind.hpp>  // 用于绑定函数和参数

void recvTopicCallback(const wired_control::PCAN_Data::ConstPtr &msg, PcanDevice &pcanDevice) {

  pcanDevice.sendPcanData(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcan_usb");  //初始化了一个ROS节点，节点的名字是 pcan_usb

  PcanDevice pcanDevice;

  ros::NodeHandle nh;
  ros::Subscriber subscriber = nh.subscribe<wired_control::PCAN_Data> ("pcan_data_send", 1000, boost::bind(&recvTopicCallback, _1, pcanDevice));
  ros::Publisher pubscriber = nh.advertise<wired_control::PCAN_Data>("pcan_data_recv", 1);

  ros::Rate loop_rate(1000); //设置循环的频率为1000Hz，即每秒循环1000次。
  wired_control::PCAN_Data pCAN_Data;
  while(ros::ok())
  {

    pcanDevice.recvPcanData(&pCAN_Data);
    pubscriber.publish(pCAN_Data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
