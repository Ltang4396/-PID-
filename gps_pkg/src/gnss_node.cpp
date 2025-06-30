#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Pose2D.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

class GNSSNode
{
public:
    GNSSNode(ros::NodeHandle& nh)
        : nh_(nh), serial_(), ref_received_(false), ref_param_written_(false),
        ref_ecef_x_(0.0), ref_ecef_y_(0.0), ref_ecef_z_(0.0),
        ref_longitude_(0.0), ref_latitude_(0.0)
    {
        try
        {
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(230400);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
            serial_.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("Unable to open port /dev/ttyUSB0");
            throw e;
        }

        if (!serial_.isOpen())
        {
            ROS_ERROR("Serial port not opened");
            throw std::runtime_error("Serial port not opened");
        }
        ROS_INFO("Serial port opened successfully");

        pub_pose_xy_ = nh_.advertise<geometry_msgs::Pose2D>("pose_xy", 10);
        pub_pose_LLA_ = nh_.advertise<geometry_msgs::Pose2D>("pose_LLA", 10);
        pub_gnss_relative_ = nh_.advertise<geometry_msgs::Pose2D>("gnss_relative", 10);
        pose_angle_pub_ = nh_.advertise<geometry_msgs::Pose2D>("pose_angle", 10);

        sub_pose_xy_ = nh_.subscribe("pose_xy", 100, &GNSSNode::callbackPoseXY, this);
        sub_pose_LLA_ = nh_.subscribe("pose_LLA", 100, &GNSSNode::callbackPoseLLA, this);

        file_.open("detail_output.txt", std::ios::app);
        if (!file_.is_open())
        {
            ROS_ERROR("Failed to open detail_output.txt");
            throw std::runtime_error("Failed to open detail_output.txt");
        }
    }

    ~GNSSNode()
    {
        if (file_.is_open())
            file_.close();
    }

    void run()
    {
        ros::Rate rate(10);

        nh_.getParam("ref_ecef_x", ref_ecef_x_);
        nh_.getParam("ref_ecef_y", ref_ecef_y_);
        nh_.getParam("ref_ecef_z", ref_ecef_z_);
        nh_.getParam("longitude", ref_longitude_);
        nh_.getParam("latitude", ref_latitude_);

        const double lon_rad = ref_longitude_ * M_PI / 180.0;
        const double lat_rad = ref_latitude_ * M_PI / 180.0;
        const double sin_lon = sin(lon_rad);
        const double cos_lon = cos(lon_rad);
        const double sin_lat = sin(lat_rad);
        const double cos_lat = cos(lat_rad);

        std::string buffer;

        while (ros::ok())
        {
            if (serial_.available())
            {
                buffer += serial_.read(serial_.available());

                size_t start = buffer.find("$G");
                size_t end = buffer.find("\r\n", start);
                while (start != std::string::npos && end != std::string::npos)
                {
                    std::string frame = buffer.substr(start, end + 2 - start);

                    if (frame.find("$GPXYZ") != std::string::npos)
                        processGPXYZ(frame);
                    else if (frame.find("$GPCHC") != std::string::npos)
                        processGPCHC(frame);

                    buffer = buffer.substr(end + 2);
                    start = buffer.find("$G");
                    end = buffer.find("\r\n", start);
                }
            }

            // ✅ 写入参考坐标
            if (ref_received_ && !ref_param_written_)
            {
                nh_.setParam("ref_ecef_x", ref_ecef_x_);
                nh_.setParam("ref_ecef_y", ref_ecef_y_);
                nh_.setParam("ref_ecef_z", ref_ecef_z_);
                nh_.setParam("longitude", ref_longitude_);
                nh_.setParam("latitude", ref_latitude_);

                ROS_INFO("Reference ECEF: x=%.3f y=%.3f z=%.3f", ref_ecef_x_, ref_ecef_y_, ref_ecef_z_);
                ROS_INFO("Reference LLA: longitude=%.6f latitude=%.6f", ref_longitude_, ref_latitude_);

                ref_param_written_ = true;
            }

            if (pose_xy_received_ && pose_LLA_received_)
            {
                double x_rel, y_rel;
                ecefToENU(last_ecef_x_ - ref_ecef_x_,
                    last_ecef_y_ - ref_ecef_y_,
                    last_ecef_z_ - ref_ecef_z_,
                    sin_lat, cos_lat, sin_lon, cos_lon,
                    x_rel, y_rel);

                geometry_msgs::Pose2D relative_pose;
                relative_pose.x = x_rel;
                relative_pose.y = y_rel;
                pub_gnss_relative_.publish(relative_pose);

                ROS_INFO("ENU Relative Position: x=%.3f m, y=%.3f m", x_rel, y_rel);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;

    ros::Publisher pub_pose_xy_;
    ros::Publisher pub_pose_LLA_;
    ros::Publisher pub_gnss_relative_;
    ros::Publisher pose_angle_pub_;

    ros::Subscriber sub_pose_xy_;
    ros::Subscriber sub_pose_LLA_;

    serial::Serial serial_;
    std::ofstream file_;

    bool ref_received_;
    bool ref_param_written_;
    double ref_ecef_x_, ref_ecef_y_, ref_ecef_z_;
    double ref_longitude_, ref_latitude_;

    bool pose_xy_received_ = false;
    bool pose_LLA_received_ = false;
    double last_ecef_x_ = 0.0, last_ecef_y_ = 0.0, last_ecef_z_ = 0.0;

    void processGPXYZ(const std::string& frame)
    {
        std::vector<std::string> fields = split(frame, ',');
        if (fields.size() < 5) return;

        try
        {
            double x = stringToNum<double>(fields[2]);
            double y = stringToNum<double>(fields[3]);
            double z = stringToNum<double>(fields[4]);

            geometry_msgs::Pose2D pose;
            pose.x = x;
            pose.y = y;
            pose.theta = z;
            pub_pose_xy_.publish(pose);

            last_ecef_x_ = x;
            last_ecef_y_ = y;
            last_ecef_z_ = z;
            pose_xy_received_ = true;

            file_ << ros::Time::now() << "\tGPXYZ: x=" << x << ", y=" << y << ", z=" << z << std::endl;
            file_.flush();

            //ROS_INFO("GPXYZ Received: x=%.3f y=%.3f z=%.3f", x, y, z);
        }
        catch (...) { ROS_WARN("GPXYZ parse error"); }
    }

    void processGPCHC(const std::string& frame)
    {
        std::vector<std::string> fields = split(frame, ',');
        if (fields.size() < 19) return;

        try
        {
            double heading = stringToNum<double>(fields[3]);
            double pitch = stringToNum<double>(fields[4]);
            double roll = stringToNum<double>(fields[5]);
            double latitude = stringToNum<double>(fields[12]);
            double longitude = stringToNum<double>(fields[13]);
            double altitude = stringToNum<double>(fields[14]);

            geometry_msgs::Pose2D pose_angle;
            pose_angle.x = heading;
            pose_angle.y = pitch;
            pose_angle.theta = roll;
            pose_angle_pub_.publish(pose_angle);

            geometry_msgs::Pose2D pose;
            pose.x = longitude;
            pose.y = latitude;
            pose.theta = altitude;
            pub_pose_LLA_.publish(pose);

            ref_longitude_ = longitude;
            ref_latitude_ = latitude;
            pose_LLA_received_ = true;

            file_ << ros::Time::now() << "\tGPCHC: heading=" << heading
                << ", pitch=" << pitch << ", roll=" << roll
                << ", latitude=" << latitude << ", longitude=" << longitude
                << ", altitude=" << altitude << std::endl;
            file_.flush();

            //ROS_INFO("GPCHC Received: lat=%.6f lon=%.6f alt=%.3f", latitude, longitude, altitude);
        }
        catch (...) { ROS_WARN("GPCHC parse error"); }
    }

    void callbackPoseXY(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        if (!ref_received_)
        {
            ref_ecef_x_ = msg->x;
            ref_ecef_y_ = msg->y;
            ref_ecef_z_ = msg->theta;
            checkRefReceived();
        }
    }

    void callbackPoseLLA(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        if (!ref_received_)
        {
            ref_longitude_ = msg->x;
            ref_latitude_ = msg->y;
            checkRefReceived();
        }
    }

    void checkRefReceived()
    {
        if ((ref_ecef_x_ != 0 || ref_ecef_y_ != 0 || ref_ecef_z_ != 0) &&
            (ref_longitude_ != 0 && ref_latitude_ != 0))
        {
            ref_received_ = true;
        }
    }

    std::vector<std::string> split(const std::string& str, char delim)
    {
        std::vector<std::string> elems;
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, delim))
            elems.push_back(item);
        return elems;
    }

    void ecefToENU(double dx, double dy, double dz,
        double sin_lat, double cos_lat,
        double sin_lon, double cos_lon,
        double& east, double& north)
    {
        east = -sin_lon * dx + cos_lon * dy;
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_node");
    ros::NodeHandle nh;

    GNSSNode gnss_node(nh);
    gnss_node.run();

    return 0;
}
