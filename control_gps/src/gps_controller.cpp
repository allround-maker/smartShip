#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#define Pi 3.141591
//#define RAD2DEG(x) ((x)*180./M_PI)
using namespace std;

class GpsController
{
private:
	ros::NodeHandle nh;
  	ros::Publisher gps_pub;
  	ros::Publisher gps_length_pub;
  
	ros::Timer param_update2;
	ros::Subscriber gps_sub;
	ros::Subscriber RPY_sub;
	ros::Subscriber sub_fix;
	//ros::Subscriber lidar_sub;
	
	// Gps_data - Publishing
	std_msgs::Float32MultiArray msg;
	std_msgs::Float32 gps_length;
	
	// My Location
	int k = 0;
	int start = 0;
	int yaw_start_flag = 0;
	int flag, flag1, flag2, flag3, flag4 = 0;
	double start_x = 0;
	double start_y = 0;

	double init_yaw = 0;
	double init_longitude = 0;
	double init_latitude = 0;

	//double init_lidar_left = 0;
	
	enum Sequence {GOAL1=1, GOAL2, GOAL3, GOAL4, RETURN};
	
	// Pose_ekf - delta_pos x, y, z
	// Gps Vector 3 Variable,
	float gps_x, gps_y, gps_z;
	
	
	// Theta & Target angle
	float theta, target_angle;
	double my_x, my_y, my_direction;
	double target_x, target_y;
	double target_longitude, target_latitude;
	
	double total_length = 0;
	double delta_PointX = 0;
	double delta_PointY = 0;
	
	// RPY - Roll & Pitch & Yaw - Subscribing
	float roll, pitch, yaw;
	
	
public:
  GpsController()
  {
  	gps_pub = nh.advertise<std_msgs::Float32>("/target_heading", 1);
  	gps_length_pub = nh.advertise<std_msgs::Float32>("/gps_length", 1);

  	//plan_sub = nh.subscribe("/target_change_order", 1, &GpsController::orderCallback, this);
  	gps_sub = nh.subscribe("/pose_ekf/gps_path", 1, &GpsController::gpsCallback, this);
	RPY_sub = nh.subscribe("/RPY",1, &GpsController::rpyCallback, this);
	sub_fix = nh.subscribe("/ublox/fix", 1, &GpsController::fixCallback, this);
	//lidar_sub = nh.subscribe("/scan",100, &GpsController::scanCallback, this);
  }
  
  	// Find Theta, Theta Distance & Publishing
  	void gpsLengthPublisher(float length);
  	void thetaCalculator(float my_x, float my_y);
	void thetaPublisher(float theta);

	// GPS, Coordinate, RPY & Subscrbing
	void fixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void gpsCallback(const nav_msgs::Path& input);
	void rpyCallback(const std_msgs::Float32MultiArray::ConstPtr& input);
	//void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	
};

void GpsController::gpsLengthPublisher(float length)
{	
	gps_length.data = length;
	gps_length_pub.publish(gps_length);
}

void GpsController::fixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	if(init_latitude == 0){
		init_latitude = msg->latitude;
		/*
		if(start == 0){
			start_latitude = msg->latitude;
		}*/
	}
	if(init_longitude == 0){
		init_longitude = msg->longitude;
		/*
		if(start == 0){
			start_longitude = msg->longitude;
			start = 1;
		}*/
		ROS_WARN("init gps pos");
	}
  	double lat = msg->latitude;
  	double lon= msg->longitude;

	delta_PointX = (init_longitude - lon)*88800;
	delta_PointY = (init_latitude - lat)*111000;
    
	thetaCalculator(delta_PointX, delta_PointY);
	gpsLengthPublisher(delta_PointY);

}

void GpsController::thetaCalculator(float my_x, float my_y)
{
	msg.data.clear();
	
	//my_direction = yaw;
	if(yaw_start_flag == 0){
		init_yaw = yaw;
		my_direction = my_direction - init_yaw;
		yaw_start_flag = 1;
	}

	//my_direction = my_direction - init_yaw;
	
	/*if(k == 1 || k == 2 || k == 3 || k == 4) {
		target_x = (abs(target_longitude - init_longitude))*88800;
		target_y = (abs(target_latitude - init_latitude))*111000;
		ROS_INFO("Target_y : [%f]", target_y);
	}

	if(k == 5){
		target_x = (abs(target_longitude - start_longitude))*88800;
		target_y = (abs(target_latitude - start_latitude))*111000;
		ROS_INFO("Target_y : [%f]", target_y);

	}*/
	// target_y == distance for distance y value (meter)
	
	// theta_distance = target_y
	// absolute_theta(stadium vertical theta)

	
	/*if(target_x > my_x && target_y > my_y)
	{
		theta = (-1)*atan((target_x - my_x)/(target_y - my_y))*180/Pi;
	}
	else if(target_x > my_x && target_y < my_y)
	{
		theta = -180 + atan((target_x - my_x)/(my_y - target_y))*180/Pi;
	}
	else if(target_x < my_x && target_y > my_y)
	{
		theta = atan((my_x - target_x)/(target_y - my_y))*180/Pi;
	}
	else
	{
		theta = 180 - atan((my_x - target_x)/(target_y - my_y))*180/Pi;
	}*/


	if(target_x > my_x && target_y > my_y)
		theta = -atan((target_x - my_x)/(target_y - my_y))*(180/Pi);
	else if(target_x > my_x && target_y < my_y)
		theta = -180 + atan((target_x - my_x)/(my_y - target_y))*(180/Pi);
	else if(target_x < my_x && target_y > my_y)
		theta = atan((my_x - target_x)/(target_y - my_y))*(180/Pi);
	else if(target_x < my_x && target_y < my_y)
		theta = atan((my_x - target_x)/(my_y - target_y))*(180/Pi);

	float ddd = 0.8;
	if(((target_x - ddd) < my_x) 
		&& (my_x < (target_x + ddd))
		&& ((target_y - ddd) < my_y)		
		&& (my_y < (target_y + ddd)))
	{
		ROS_WARN("==========================PASS==========================");
		k++;
		ROS_INFO("k : [%d]",k);
	}else{
		ROS_WARN("Not yet");
	}
	
	target_angle = theta - my_direction;
	thetaPublisher(theta);
	
	ROS_INFO("                                                                             ");
	ROS_INFO("Publishing GPS Data");
	ROS_INFO("yaw_start_flag : [%d], flag : [%d]", yaw_start_flag, flag);
	ROS_INFO("target_x : [%f], my_x : [%f]", target_x, my_x);
	ROS_INFO("target_y : [%f], my_y : [%f]", target_y, my_y);
	ROS_INFO("remain_distance : [%f]", sqrt(pow(target_x-my_x,2)+pow(target_y-my_y,2)));
	ROS_INFO("target_angle : [%f], target theta : [%f], my_dir : [%f]", target_angle, theta, my_direction);


	switch(Sequence(k)) {
		case Sequence::GOAL1:
			// k = 0
			if(flag == 0){
				//init_longitude = 0;
				//init_latitude = 0;

				yaw_start_flag = 0;
				flag = 1;
			}
			target_x = -3;
			target_y = 12;
			ROS_INFO("START POINT -> GOAL1");
			break;
				
		case Sequence::GOAL2:
			// k = 1
			if(flag1 == 0){
				//init_longitude = 0;
				//init_latitude = 0;
				//total_length += target_y;

				yaw_start_flag = 0;
				flag1 = 1;
			}
			target_x = 1;
			target_y = 27;
			ROS_INFO("GOAL1 -> GOAL2");
			break;
			
		case Sequence::GOAL3:
			// k = 2
			if(flag2 == 0){
				//init_longitude = 0;
				//init_latitude = 0;
				//total_length += target_y;

				yaw_start_flag = 0;
				flag2 = 1;
			}
			target_x = 4;
			target_y = 8;
			ROS_INFO("GOAL2 -> GOAL3");
			break;

		case Sequence::GOAL4:
			// k = 3
			if(flag3 == 0){
				//init_longitude = 0;
				//init_latitude = 0;
				//total_length += target_y;

				yaw_start_flag = 0;
				flag3 = 1;
			}
			target_x = 4;
			target_y = 19;
			ROS_INFO("GOAL3 -> GOAL4");
			break;
		
		case Sequence::RETURN:
			// k = 4
			if(flag4 == 0){
				//init_longitude = 0;
				//init_latitude = 0;
				//total_length += target_y;

				yaw_start_flag = 0;
				flag4 = 1;
			}
			target_x = start_x;
			target_y = start_y;
			//ROS_INFO("Total_length : [%f], Target_length : [%f]", total_length, target_y);
			ROS_INFO("GOAL4 -> START POINT");
			break;
			
		default:
			// k = upper than 3
			// STOP COMMAND
			ROS_INFO("FINISH");
			break;
	}
	ROS_INFO("                                                                             "); 
	ROS_INFO("----------------------------------------------------------------------------------");

	
}


void GpsController::thetaPublisher(float theta)
{
	msg.data.clear();
	std_msgs::Float32 msg;
	msg.data = theta;

	
	gps_pub.publish(msg);
}

void GpsController::gpsCallback(const nav_msgs::Path& input)
{
	nav_msgs::Path myPath = input;
	gps_x = myPath.poses.back().pose.position.x;
	gps_y = myPath.poses.back().pose.position.y;
		
	//ROS_INFO("Gps_x : [%f], Gps_y : [%f]", gps_x, gps_y);
	thetaCalculator(gps_y, gps_x);
}

void GpsController::rpyCallback(const std_msgs::Float32MultiArray::ConstPtr& input)
{
	roll = input->data[0];
	pitch = input->data[1];
	yaw = input->data[2];
	
	//ROS_INFO("Yaw : [%f]", yaw);
}
/*
void GpsController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int count = scan->scan_time / scan->time_increment;
    //printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    //printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
  	for(int i = 0; i < count; i++) {
  		float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		if(degree > -30 && degree< 30){
    		printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
   	}
  }
}*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_controller");
	GpsController gps;
	ros::spin();
	return 0;
}
