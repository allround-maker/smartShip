#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include <cmath>

using namespace std;

class ReachTheTarget
{
private:
  ros::NodeHandle nh;
  ros::Publisher target_change_pub;
	ros::Subscriber target_data_sub;

  std_msgs::Bool order;
  
  float startPointX, startPointY;
  float targetPointX, targetPointY;
	
public:
  ReachTheTarget()
  {
  	target_change_pub = nh.advertise<std_msgs::Float32MultiArray>("/target_change_order", 1);
  	target_data_sub = nh.subscribe("/vector_data", 1, &GpsController::targetCallback, this);  	
  }
  void targetdataCallback(const std_msgs::Float32MultiArray::ConstPtr& input);
};

void ReachTheTarget::targetdataCallback(const std_msgs::Float32MultiArray::ConstPtr& input){
	startPointX = input->data[0];
	startPointY = input->data[1];
	targetPointX = input->data[2];
	targetPointY = input->data[3];
	
	//if - else문
	
	// 도달 했을 경우
	if(도달) {
		order.data = true;
	} else {
		order.data = false;
	}
	order.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ReachTheTarget RTT;
  ros::spin();
  return 0;
}

