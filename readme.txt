本项目采用了PID控制车辆油门开度，使得其能够达到目标车速

该项目主要通过两部分完成：
1.通过gps_pkg文件夹下的gnss_node2gai.cpp文件，输出当前车辆的车速(通过“pose_velocity”话题发布，消息类型为geometry_msgs::Pose2D，其中的Pose2D.theta对应车辆行进车速)。
2.通过wired_control文件夹下的pid_throttle_control2.cpp文件，其根据我们输入的目标车速，以及车辆的当前车速，根据PID算法输出对应的油门开度，将油门开度转换成CAN协议的格式输送给CAN。

运行代码顺序如下(进入文件夹以及source ./devel/setup.bash的步骤在此不说明)：
1.roscore
2.rosrun gps_pkg gnss_node2gai
3.rosrun wired_control pcan_usb
4.rosrun wired_control mode_set_gai  0 0 0 1 0  (此处只是我们实验室的车如此，具体见mode_set_gai代码)
5.rosrun wired_control pid_throttlec_control2 x (x 对应目标车速，在本项目中只试验了车速为2m/s时候的情况)

补充说明：
其中的部分CMakeLists文件可能会报错，因为后续没从实车的代码中将CMakeLists文件拷贝回来
