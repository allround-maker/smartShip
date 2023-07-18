#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Vector3Stamped.h>
#include "control_pkg/order.h" 
using namespace std;

class GpsPublisher
{
private:
  ros::NodeHandle nh;
  ros::Publisher gps_pub;

  float x;
  float y;
  float z;
	
	string fixed_frame_id;	
	  
  vector<float> point;

public:
  GpsPublisher()
  {
  	fixed_frame_id = "pos";
  	gps_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/delta_pos", 1);
  }
  
  void pub(){
  	geometry_msgs::Vector3Stamped pos;
  	point = {10,20,30};

		pos.header.frame_id = fixed_frame_id;
		pos.vector.x = point[0];
		pos.vector.y = point[1];
		pos.vector.z = point[2];

		gps_pub.publish(pos);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_publisher");
  GpsPublisher gps;
  ros::spin();
  return 0;
}
