#include "ros/ros.h"                            // ROS Default Header File
#include "std_msgs/Int32MultiArray.h"
#include "control_pkg/order.h" 

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "test_order_pub");     // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system
  float pubT = 0.1;
  nh.getParam("test_order_pub/pubT", pubT);
  // Declare publisher, create publisher 'ros_tutorial_pub' using the 'MsgTutorial'
  // message file from the 'ros_tutorials_topic' package. The topic name is
  // 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
  ros::Publisher pub = nh.advertise<control_pkg::order>("/run_order", 100);

  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
  ros::Rate loop_rate(pubT);

  
  int count = 0;                            // Variable to be used in message
  int Right_motor=0;
  int Left_motor=0;

  int mode = 1;
  float time = 5;
  int R_motor = 100;
  int L_motor= 100;
  float yaw =0;
  bool smooth = 0;
  float dis =0;
  
  //nh.getParam("test_order_pub/mode", mode);
  //nh.getParam("test_order_pub/R", R_motor);
  //nh.getParam("test_order_pub/L", L_motor);
  //nh.getParam("test_order_pub/time", time);
  //nh.getParam("test_order_pub/yaw", yaw);
  //nh.getParam("test_order_pub/smooth", smooth);
  //nh.getParam("test_order_pub/dis", dis);
  
  while (ros::ok())
  {
	control_pkg::order test_order;
	//Motor_value.data.push_back(1550);
    //Motor_value.data.push_back(1550);
    test_order.mode = mode;
    test_order.time = time;
    test_order.R =R_motor;
    test_order.L =L_motor;
    test_order.yaw=yaw;
    test_order.smooth=smooth;
    test_order.dis=dis;
    //ROS_INFO("Right_motor %d, %d", mode, Left_motor);
    pub.publish(test_order);          // Publishes 'msg' message
    loop_rate.sleep();                      // Goes to sleep according to the loop rate defined above.
                                // Increase count variable by one
    
  }
  return 0;
}


