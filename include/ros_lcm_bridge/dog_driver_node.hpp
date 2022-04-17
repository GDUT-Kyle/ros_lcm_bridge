#ifndef DOG_DRIVER_NODE_H
#define DOG_DRIVER_NODE_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <string>
#include <mutex>
#include <vector>
#include <tf/transform_broadcaster.h>
// #include <serial/serial.h>
#include <lcm/lcm-cpp.hpp>
#include "doglcm/rec_state.hpp"
#include "doglcm/pub_ctrl.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace dog_driver
{
// #define CONTROL_VEL std::string("c")
// #define READ_VEL    std::string("v")

class DogDriverNode
{
public:
    // ros
    ros::NodeHandle n_;
    ros::Publisher pub_odom;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_reset;

    // lcm
    lcm::LCM my_lcm;
    int framerate_;
    std::string dog_device_;

    std::mutex mtx;

    // 接受的数据
    std::vector<double> recVelocity;
    std::vector<double> lastRecVelocity;
    Eigen::Matrix<double, 6, 1> dMotion;

    Eigen::Vector3d curPosition;
    Eigen::Quaterniond curPose;
    double lastTime = 0.0;
    double curTime = 0.0;
    double dt = 0.0;
    bool initTime = false;

    double lastMovingTime = 0.0;
    bool isMoving = false;

    nav_msgs::Odometry odomMsg;

public:
    DogDriverNode();
    virtual ~DogDriverNode();
    void config();
    void connect();
    void disconnect();
    void stop();
    void spin();
    void checkPort();
    void parseOdometry();
    void PublishOdometryToROS();
    void PublishTF();
    void setVelocity(std::vector<double>& linear_vel, std::vector<double>& angular_vel);
    void cmdVelHandler(const geometry_msgs::Twist::ConstPtr cmdVel);
    void ResetOdomIntegratorCallback(const std_msgs::Bool::ConstPtr& msg);
    void lcm_handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
                            const doglcm::rec_state* msg);
};
} // namespace name


#endif