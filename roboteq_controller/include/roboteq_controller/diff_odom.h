#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>
#include <roboteq_controller/channel_values.h>
#include <roboteq_controller/command_srv.h>

/*
class Odometry
{
public:
	Odometry(ros::NodeHandle, ros::NodeHandle);
	~Odometry(){
		if (ser_.isOpen()){
			ser_.close();
		}
	}

private:

};
*/



class Odometry{

public:
	Odometry();

	void spin();


private:
	ros::NodeHandle nh;
	ros::ServiceClient command_client;
	roboteq_controller::command_srv command_service;
	ros::Subscriber sub;
	ros::Subscriber l_wheel_sub;
	ros::Subscriber r_wheel_sub;
	ros::Subscriber wheels_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_lencoder;
	double prev_rencoder;

	double lmult;
	double rmult;

	double left;
	double right;

	int ppr;

	bool publish_tf;
	bool get_odom;

	double rate;

	double radius, gear_ratio;

	std::string encoder_topic;
	std::string odom_frame;
	std::string base_frame;
	std::string command_srv;

	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;


	double enc_left ;

	double enc_right;

	double ticks_meter;

	double base_width;

	double dx;

	double dr;

	double x_final,y_final, theta_final;

	ros::Time current_time, last_time;


	void encoderCb(const roboteq_controller::channel_values& ticks);
	//void rightencoderCb(std_msgs::Int64::ConstPtr& right_ticks);
	void init_variables();

	void get_node_params();


	void update();
};

