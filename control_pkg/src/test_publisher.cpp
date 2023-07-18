#include "ros/ros.h"                            // ROS Default Header File
#include "std_msgs/Int32MultiArray.h"

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "motor_test_publisher");     // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system

  // Declare publisher, create publisher 'ros_tutorial_pub' using the 'MsgTutorial'
  // message file from the 'ros_tutorials_topic' package. The topic name is
  // 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
  ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("Motor_run", 100);

  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
  ros::Rate loop_rate(1);

  
  int count = 0;                            // Variable to be used in message
  int Right_motor;
  int Left_motor;
  
  nh.getParam("/serial_node/Right_motor", Right_motor);
  nh.getParam("/serial_node/Left_motor", Left_motor);
  while (ros::ok())
  {
	std_msgs::Int32MultiArray Motor_value;
	//Motor_value.data.push_back(1550);
    //Motor_value.data.push_back(1550);
    Motor_value.data.push_back(Right_motor);
    Motor_value.data.push_back(Left_motor);
    //ROS_INFO("Right_motor %d, %d", Right_motor, Left_motor);
    pub.publish(Motor_value);          // Publishes 'msg' message
	ROS_INFO("I publish %d, %d", Motor_value.data[0], Motor_value.data[1]);
    loop_rate.sleep();                      // Goes to sleep according to the loop rate defined above.
                                // Increase count variable by one
  }
  return 0;
}


