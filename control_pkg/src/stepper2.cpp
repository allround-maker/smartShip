#include "ros/ros.h"                            // ROS Default Header File
#include "std_msgs/Int32.h"

int doc_point ;

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "stepper");     // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system

  // Declare publisher, create publisher 'ros_tutorial_pub' using the 'MsgTutorial'
  // message file from the 'ros_tutorials_topic' package. The topic name is
  // 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/run_step", 1);
  //ros::Subscriber sub = nh.subscribe("/docking_point", 1, docCallback);
  ros::Rate loop_rate(1);
  
  while(!nh.getParam("/dock_point", doc_point)){
    loop_rate.sleep();
  }

  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
  std_msgs::Int32 msg;
  
  for(int i = 0; i< 3; i++){
    loop_rate.sleep();
  }
  msg.data = doc_point+10;
  pub.publish(msg);
  ROS_WARN("start");
  for(int i = 0; i< 110; i++){
    loop_rate.sleep();
  }

  return 0;
}


