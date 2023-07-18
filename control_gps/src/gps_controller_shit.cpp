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
#define RAD2DEG(x) ((x)*180./M_PI)
using namespace std;

class GpsController
{
private:
	ros::NodeHandle nh;
  	ros::Publisher gps_pub;
  	ros::Publisher gps_length_pub;
  
	ros::Subscriber gps_sub;
	ros::Subscriber RPY_sub;
	ros::Subscriber sub_fix;
	ros::Subscriber lidar_sub;
	
	// Gps_data - Publishing
	std_msgs::Float32MultiArray msg;
	std_msgs::Float32 gps_length;
	
	// My Location
	int k = 0;
	int start = 0;
	int flag, flag1, flag2, flag3 = 0;
	double start_longitude = 127.287035;
	double start_latitude = 36.6213463;

	double init_longitude = 0;
	double init_latitude = 0;

	double init_lidar_left = 0;
	
	enum Sequence {GOAL1, GOAL2, GOAL3, RETURN};
	
	// Pose_ekf - delta_pos x, y, z
	// Gps Vector 3 Variable,
	float gps_x, gps_y, gps_z;
	
	
	// Theta & Target angle
	float theta, target_angle;
	float my_x, my_y, my_direction;
	double target_x, target_y;
	double target_longitude, target_latitude;
	vector<float> direction;
	
	
	double total_length = 0;
	double delta_PointX = 0;
	double delta_PointY = 0;
	
	// RPY - Roll & Pitch & Yaw - Subscribing
	float roll, pitch, yaw;
	
	
public:
  GpsController()
  {
  	gps_pub = nh.advertise<std_msgs::Float32MultiArray>("/gps_data", 1);
  	gps_length_pub = nh.advertise<std_msgs::Float32>("/gps_length", 1);
  	
  	//plan_sub = nh.subscribe("/target_change_order", 1, &GpsController::orderCallback, this);
  	gps_sub = nh.subscribe("/pose_ekf/gps_path", 1, &GpsController::gpsCallback, this);
	RPY_sub = nh.subscribe("/RPY",1, &GpsController::rpyCallback, this);
	sub_fix = nh.subscribe("/ublox/fix", 100, &GpsController::fixCallback, this);
	lidar_sub = nh.subscribe("/scan",100, &GpsController::scanCallback, this);
  }
  
  	// Find Theta, Theta Distance & Publishing
  	void gpsLengthPublisher(float length);
  	void thetaCalculator(float my_x, float my_y);
	void thetaPublisher(float theta, float target_angle);
	
	// GPS, Coordinate, RPY & Subscrbing
	void fixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void gpsCallback(const nav_msgs::Path& input);
	void rpyCallback(const std_msgs::Float32MultiArray::ConstPtr& input);
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	
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
		if(start == 0){
			start_latitude = msg->latitude;
		}
	}
	if(init_longitude == 0){
		init_longitude = msg->longitude;
		if(start == 0){
			start_longitude = msg->longitude;
			start = 1;
		}
		ROS_WARN("init gps pos");
	}
  	double lat = msg->latitude;
  	double lon= msg->longitude;

	delta_PointX = (init_longitude - lon)*88800;
	delta_PointY = (init_latitude - lat)*111000;
    
	thetaCalculator(-delta_PointX, -delta_PointY);
	gpsLengthPublisher(delta_PointY);

}

void GpsController::thetaCalculator(float my_x, float my_y)
{
	msg.data.clear();
	my_direction = yaw;
	vector<float> direction = {14, -15, 9, 0, -11}; 
	k = 0;
	if( )
		

}


void GpsController::thetaPublisher(float theta, float target_angle)
{
	msg.data.clear();
	
	msg.data.push_back(theta);
	msg.data.push_back(target_angle);
	
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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_controller");
	GpsController gps;
	ros::spin();
	return 0;
}
