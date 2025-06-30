# 🚗 PID 控制车辆油门项目

本项目采用 **PID 控制算法** 实现车辆油门的自动调节，使车辆能够精准达到设定的目标车速。

---

## 📦 项目模块说明

本项目主要由两部分组成：

1. **GNSS 车速获取模块**  
   位于 gps_pkg 文件夹中的 gnss_node2gai.cpp 文件：  
   - 输出当前车辆车速；
   - 发布话题：`pose_velocity`
   - 消息类型：`geometry_msgs::Pose2D`
   - 其中 Pose2D.theta 即为当前车速（单位 m/s）。

2. **油门控制模块（PID 控制）**  
   位于 wired_control 文件夹中的 pid_throttle_control2.cpp 文件：  
   - 接收目标车速 + 当前车速；
   - 使用 PID 算法计算油门开度；
   - 输出油门 CAN 报文，发送给 CAN 总线。
---
## 🧪 运行步骤（示例流程）
> 💡 假设你已完成 `catkin_make` 并执行了 `source ./devel/setup.bash`
依次运行如下命令：
```bash
1️⃣ roscore
2️⃣ rosrun gps_pkg gnss_node2gai
3️⃣ rosrun wired_control pcan_usb
4️⃣ rosrun wired_control mode_set_gai 0 0 0 1 0  # ⚠️ 本实验室车辆所用配置，其他车请参考代码
5️⃣ rosrun wired_control pid_throttle_control2 2  # 🚀 目标车速设为 2.0 m/s
```

---
## 📌 补充说明

1.当前仅在 目标速度为 2 m/s 的情况下进行了实车测试；  
2.某些 CMakeLists.txt 文件可能存在缺失或配置错误，这是由于未从实车环境中完整拷贝回来，如遇构建错误请手动补齐依赖。

## 🧠 PID 控制说明（简要）
PID 控制器通过以下公式计算油门输出：  
output = Kp * error + Ki * ∫error dt + Kd * d(error)/dt  
其中：  
error = 目标车速 - 当前车速  
Kp：比例系数，决定反应速度  
Ki：积分系数，消除稳态误差  
Kd：微分系数，抑制系统震荡
