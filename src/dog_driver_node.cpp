#include "ros_lcm_bridge/dog_driver_node.hpp"
#include "ros_lcm_bridge/util.h"

using namespace dog_driver;

DogDriverNode::DogDriverNode() : n_("~")
{
	n_.param("dog_device", dog_device_, std::string("rec_dog_lcm"));
	n_.param("framerate", framerate_, 100);

	recVelocity.resize(6);
	lastRecVelocity.resize(6);
	dMotion.resize(6);
	curPosition.setZero();
	curPose.setIdentity();

    if(!my_lcm.good())
        return;

    my_lcm.subscribe(dog_device_, &DogDriverNode::lcm_handleMessage, this);

	pub_odom = n_.advertise<nav_msgs::Odometry>("/odom", 5);
	sub_vel = 
			n_.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, boost::bind(&DogDriverNode::cmdVelHandler, this, _1));
	sub_reset =
			n_.subscribe<std_msgs::Bool>("/dog/reset", 1, boost::bind(&DogDriverNode::ResetOdomIntegratorCallback, this, _1));

    stop();
	lastMovingTime = ros::Time::now().toSec();
	isMoving = false;

	// check conection parameter
	ROS_INFO("The channel subscribed by LCM is \033[1;32;40m'%s'", dog_device_.c_str());
}

DogDriverNode::~DogDriverNode()
{
	stop();
}

void DogDriverNode::lcm_handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
                            const doglcm::rec_state* msg)
{
    for(int i=0; i<3; i++)
    {
        recVelocity[i] = msg->linear_velocity[i];
        recVelocity[i+3] = msg->angular_velocity[i];
    }
    parseOdometry();
}

void DogDriverNode::cmdVelHandler(const geometry_msgs::Twist::ConstPtr cmdVel)
{
	// mtx.lock();
    std::vector<double> linear_vel = {cmdVel->linear.x, cmdVel->linear.y, cmdVel->linear.z};
    std::vector<double> angular_vel = {cmdVel->angular.x, cmdVel->angular.y, cmdVel->angular.z};
    setVelocity(linear_vel, angular_vel);
	lastMovingTime = ros::Time::now().toSec();
	isMoving = true;
	// mtx.unlock();
}

// unused
void DogDriverNode::ResetOdomIntegratorCallback(const std_msgs::Bool::ConstPtr& msg)
{
	// mtx.lock();
	if(msg->data)
	{
		curPosition.setZero();
		curPose.setIdentity();
	}
	// mtx.unlock();
}

void DogDriverNode::setVelocity(std::vector<double>& linear_vel, std::vector<double>& angular_vel)
{
    doglcm::pub_ctrl ctrl_data;
    for(int i=0; i<3; i++)
    {
        ctrl_data.linear_velocity[i] = linear_vel[i];
        ctrl_data.angular_velocity[i+3] = angular_vel[i];
    }
    my_lcm.publish("pub_dog_lcm", &ctrl_data);
}

void DogDriverNode::stop()
{
    std::vector<double> linear_vel(3, 0.0);
    std::vector<double> angular_vel(3, 0.0);
    setVelocity(linear_vel, angular_vel);
}

void DogDriverNode::parseOdometry()
{
	if(!initTime)
	{
		initTime = true;
		curTime = ros::Time::now().toSec();
		lastTime = curTime;
		for(size_t i=0; i<recVelocity.size(); i++)
			lastRecVelocity[i] = recVelocity[i];
		return;
	}

	curTime = ros::Time::now().toSec();
	dt = curTime - lastTime;
	lastTime = curTime;
	// 中值积分
	for(size_t i=0; i<recVelocity.size(); i++)
	{
		dMotion(i, 0) = 0.5*(lastRecVelocity[i]+recVelocity[i])*dt;
	}

	lastRecVelocity.assign(recVelocity.begin(), recVelocity.end());

	Eigen::AngleAxisd dYaw(dMotion(5,0),Eigen::Vector3d(0, 0, 1));
	Eigen::AngleAxisd dPitch(dMotion(4,0),Eigen::Vector3d(0, 1, 0));
	Eigen::AngleAxisd dRoll(dMotion(3,0),Eigen::Vector3d(1, 0, 0));
	// Eigen::Quaterniond dPose = dRoll*dPitch*dYaw;
	Eigen::Quaterniond dPose(dYaw);

	Eigen::Vector3d dPosition = 1.3 * dMotion.block(0, 0, 3, 1);
	dPosition.z() = 0.0;

	curPose = curPose * dPose;
	curPosition += curPose * dPosition;

	PublishOdometryToROS();
	PublishTF();
}

void DogDriverNode::PublishOdometryToROS()
{
	odomMsg.header.stamp = ros::Time().fromSec(curTime);
	odomMsg.header.frame_id = "odom";
	odomMsg.child_frame_id = "dog_base";
	odomMsg.pose.pose.position.x = curPosition.x();
	odomMsg.pose.pose.position.y = curPosition.y();
	odomMsg.pose.pose.position.z = curPosition.z();
	odomMsg.pose.pose.orientation.w = curPose.w();
	odomMsg.pose.pose.orientation.x = curPose.x();
	odomMsg.pose.pose.orientation.y = curPose.y();
	odomMsg.pose.pose.orientation.z = curPose.z();
	pub_odom.publish(odomMsg);
}

void DogDriverNode::PublishTF()
{
	static tf::TransformBroadcaster br;
  	tf::Transform transform;
	transform.setOrigin( tf::Vector3(curPosition.x(), curPosition.y(), curPosition.z()) );
	tf::Quaternion q(curPose.x(), curPose.y(), curPose.z(), curPose.w());
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(curTime), 
						odomMsg.header.frame_id, odomMsg.child_frame_id));
}

void DogDriverNode::spin()
{
    ros::Rate loop_rate(framerate_);
    while(ros::ok())
    {
        my_lcm.handleTimeout(1);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dog_driver");
    DogDriverNode DogDriverNode_;
    DogDriverNode_.spin();
    return 0;
}