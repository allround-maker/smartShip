#include "ros/ros.h"                            // ROS Default Header File
#include "std_msgs/Int32.h"

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "stepper");     // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system

  // Declare publisher, create publisher 'ros_tutorial_pub' using the 'MsgTutorial'
  // message file from the 'ros_tutorials_topic' package. The topic name is
  // 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/run_step", 1);

  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
  ros::Rate loop_rate(1);
  std_msgs::Int32 msg;
  for(int i = 0; i< 3; i++){
    loop_rate.sleep();
  }
  msg.data = 0;
  pub.publish(msg);
  ROS_WARN("start");
  for(int i = 0; i< 11; i++){
    loop_rate.sleep();
  }
  ROS_WARN("center_Dir");
  msg.data = 1;
  pub.publish(msg);
  for(int i = 0; i< 13; i++){
    loop_rate.sleep();
  }
  ROS_WARN("center_Dir");

  msg.data = 2;
  pub.publish(msg);

  return 0;
}


