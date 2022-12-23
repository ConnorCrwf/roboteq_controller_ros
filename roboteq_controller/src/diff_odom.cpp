#include "roboteq_controller/diff_odom.h"

/*
Odometry::Odometry(ros::NodeHandle nh, ros::NodeHandle nh_priv):
	nh_(nh),
	nh_priv_(nh_priv) {

	

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	Odometry node(nh, nh_priv);
	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
	ros::waitForShutdown();

	return 0;
}
*/


Odometry::Odometry(){

	init_variables();
	ROS_INFO("Started Odometry computing node");
	wheels_sub = nh.subscribe(encoder_topic,1000, &Odometry::encoderCb, this);
  	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
	//Retrieving parameters of this node

}

void Odometry::init_variables()
{

	nh.param<std::string>("encoder_topic_name", encoder_topic, "/encoder_count");
	nh.param("reduction_ratio_internal", reduction_ratio_internal, 1.0);
	nh.param("reduction_ratio_external", reduction_ratio_external, 1.0);
	nh.param("poles", poles, 5.0);
	nh.param("radius", radius, 1.0);
	nh.param("wheelbase", base_width, 1.0);
	nh.param("rate", rate, 5.0);
	nh.param("ppr", ppr, 1024);
	nh.param("encoder_max", encoder_max, 65536.0);
	nh.param("encoder_min", encoder_min, -65536.0);
	nh.param("publish_tf", publish_tf, false);
	nh.param<std::string>("odom_frame", odom_frame, "odom");
	nh.param<std::string>("base_frame", base_frame, "base_link");
	nh.param<std::string>("command_srv", command_srv, "/dualchannel_command_service");
	command_client = nh.serviceClient<roboteq_controller::command_srv>(command_srv);
	command_service.request.channel = 0;
	command_service.request.userInput = "C";
	command_service.request.value = 0;
	command_client.call(command_service);

	prev_lencoder = 0;
	prev_rencoder = 0;

	lmult = 0;
	rmult = 0;

	left = 0;
	right = 0;

	// encoder_min =  -65536;
	// encoder_max =  65536;
	// rate = 5;
	// base_width = 1; // distance between wheels

	// calculation of ticks per meter

	reduction_ratio = reduction_ratio_internal*reduction_ratio_external;
	cpr = poles*(1/reduction_ratio);  //should come out to 245

	//temporary correction until I figure out reason for issue
	double correction_factor = 12.1245
	double cpr_new = correction_factor*cpr;

	ticks_meter = cpr_new/(2 * M_PI * radius);   // counters per revolution divided by wheel diameter;
	// ticks_meter = (ppr * gear_ratio * 4) / (2 * M_PI * radius); // 1024=PPR 24.69=Gear Ratio 4=CPR/PPR 0.105=Radius of wheel

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;

	then = ros::Time::now();
	x_final = 0 ; y_final = 0; theta_final = 0;
	enc_left = left;
	enc_right =right;

	dx = 0;
	dr = 0;
	get_odom = false;
 	current_time = ros::Time::now();
  	last_time = ros::Time::now();

}

//Spin function
void Odometry::spin(){
     ros::Rate loop_rate(rate);
     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}

//Update function
void Odometry::update(){

	ros::Time now = ros::Time::now();

	double elapsed;

	double d_left, d_right, d, th,x,y;

	if ( now > t_next) {

		elapsed = now.toSec() - then.toSec();
		if(get_odom == true){
		if(left == 0 && right == 0) {
			d_left = 0;
			d_right = 0;
			x_final = 0 ;
			y_final = 0;
			theta_final = 0;
		}
		else{
			d_left = (left - enc_left) / ( ticks_meter);
			d_right = (right - enc_right) / ( ticks_meter);
		}

		enc_left = left;
		enc_right = right;
		d = (d_left + d_right ) / 2.0;
		th = (d_left - d_right) / base_width;
		dx = d /elapsed;
		dr = th / elapsed;

		if ( d != 0){
                	x = cos( th ) * d;
                	y = sin( th ) * d;
                	// calculate the final position of the robot
                	x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
                	y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
			}
           	if( th != 0){
                	theta_final = theta_final + th;
				}
		    geometry_msgs::Quaternion odom_quat ;
		    odom_quat.x = 0.0;
		    odom_quat.y = 0.0;
		    odom_quat.z = sin( theta_final / 2 );
            odom_quat.w = cos( theta_final / 2 );

		    //first, we'll publish the transform over tf
			if (publish_tf){
				geometry_msgs::TransformStamped odom_trans;
				odom_trans.header.stamp = now;
				odom_trans.header.frame_id = odom_frame;
				odom_trans.child_frame_id = base_frame;

				odom_trans.transform.translation.x = x_final;
				odom_trans.transform.translation.y = y_final;
				odom_trans.transform.translation.z = 0.0;  // 2D Z axis is 0
				odom_trans.transform.rotation = odom_quat;

				//send the transform
				odom_broadcaster.sendTransform(odom_trans);
			}

		    //next, we'll publish the odometry message over ROS
		    nav_msgs::Odometry odom;
		    odom.header.stamp = now;
		    odom.header.frame_id = odom_frame;

		    //set the position
		    odom.pose.pose.position.x = x_final;
		    odom.pose.pose.position.y = y_final;
		    odom.pose.pose.position.z = 0.0;
		    odom.pose.pose.orientation = odom_quat;

		    //set the velocity
		    odom.child_frame_id = base_frame;
		    odom.twist.twist.linear.x = dx;
		    odom.twist.twist.linear.y = 0;
		    odom.twist.twist.angular.z = dr;

		    //publish the message
		    odom_pub.publish(odom);
		}
	    	    then = now;
	            ros::spinOnce();
		}
	 else { ; }

}
void Odometry::encoderCb(const roboteq_controller::channel_values& ticks)
{

	double l_enc = ticks.value[0];
	double r_enc = ticks.value[1];

	if((l_enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		lmult = lmult + 1;
	}

	if((r_enc < encoder_low_wrap) && (prev_rencoder > encoder_high_wrap))
	{
		rmult = rmult + 1;
	}

	if((l_enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))
	{
		lmult = lmult - 1;
	}

	if((r_enc > encoder_high_wrap) && (prev_rencoder < encoder_low_wrap))
	{
		rmult = rmult - 1;
	}
	left = 1.0 * (l_enc + lmult * (encoder_max - encoder_min));
	right = 1.0 * (r_enc + rmult * (encoder_max - encoder_min));
	prev_rencoder = r_enc;
	prev_lencoder = l_enc;
	get_odom = true;
}


int main(int argc, char **argv)

{
	ros::init(argc, argv,"diff_odom");
	Odometry obj;
	obj.spin();
	return 0;
}
