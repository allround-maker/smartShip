#include "ros/ros.h"                            // ROS Default Header File
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "control_pkg/order.h" 
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;
class DriveAlgo
{
private:

// flag
  float count_origin = 0;
  float count_down = 0;
  bool time_flag = 0;



//gps distance data
  float total_distance =0;
  float target_distance =0;
  float before_x = 0;
  float before_y = 0;


//control_parameter -100~100
  int R_output;
  int L_output;

// Limit Value : 
  int str_limit = 20;
  int turn_limit = 70;
  int break_limit = -70;
//Power Parameter
// R_ output = a
  float a=11;
  float b=5;
  float c=0.5;
  float d=8;

// break_value
  float break_distance = 1;
  float velocity;
  float FPtolidar = 0.4;
  float brake_value = -70;
// Sensor Data
  float front_dis;
  float theta;
  float theta_dis;
  float front_dis_for_break;

// Order T
  float timer_T = 0.2;
  bool smooth=0;

public:
  DriveAlgo()
  {
    pub_order = nh.advertise<control_pkg::order>("/run_order", 1);
    sub_dir = nh.subscribe("/Direction_for_avoid", 1, &DriveAlgo::dirCallback, this);
    param_update = nh.createTimer(ros::Duration(timer_T/2), &DriveAlgo::updaterCallback, this);
    timer = nh.createTimer(ros::Duration(timer_T), &DriveAlgo::timerCallback, this);
    sub_vel = nh.subscribe("/delta_vel", 1, &DriveAlgo::veloCallback, this);
  } 

  int limitValue(int input, int max, int min){
    if(input>max) return max;
    if(input<min) return min;
    return input;
  }
  float limitValue(float input, int max, int min){
    if(input>max) return max;
    if(input<min) return min;
    return input;
  } 
  float setDegree(float input){
    if(input>180) return input-360;
    if(input<-180) return input+360;
    return input;
  }
  

  void dirCallback(const std_msgs::Float32MultiArray::ConstPtr& input){
    //update data
    theta = input->data[0];
    theta_dis = input->data[1];
    front_dis = input->data[2];
    front_dis_for_break = input->data[3];
  }
  void updaterCallback(const ros::TimerEvent& event){
    // update parameter!
    nh.getParam("/control_param/a", a);
    nh.getParam("/control_param/b", b);
    nh.getParam("/control_param/c", c);
    nh.getParam("/control_param/d", d);
    nh.getParam("/control_param/smooth", smooth);
  }

  void veloCallback(const geometry_msgs::Vector3Stamped::ConstPtr& input){
    velocity = input->vector.x;
  }
  void timerCallback(const ros::TimerEvent& event){
    // It's order time!
    int break_ = 0;
    control_pkg::order order;

    if(front_dis_for_break<break_distance){
      break_ = (break_distance-front_dis_for_break)*100/(break_distance-FPtolidar)*-1;
      break_ = limitValue(break_, 0, brake_value); 
      ROS_WARN("break");
      //break_ = 0;
    
    }
      cout << "front dis (brake) : "<<front_dis_for_break << endl;

    if(theta>50){
      R_output = limitValue((c*abs(theta)+d), turn_limit,turn_limit*-1)*0.8;
      L_output = -limitValue((c*abs(theta)+d), turn_limit,turn_limit*-1)*1.2;
      ROS_WARN("point_TURN");
    }else if(theta > 0){
      R_output = limitValue(a*front_dis+b, str_limit, -1*str_limit) + limitValue((c*abs(theta)+d), turn_limit,turn_limit*-1) *0.8+ break_;
      L_output = limitValue(a*front_dis+b, str_limit, -1*str_limit) - limitValue((c*abs(theta)+d), turn_limit,-1*turn_limit)*1.2+ break_;
    }else if(theta<-50){
      R_output = -limitValue((c*abs(theta)+d), turn_limit,turn_limit*-1)*1.2;
      L_output = limitValue((c*abs(theta)+d), turn_limit,turn_limit*-1)*0.8;
      ROS_WARN("point_TURN");
    }else if(theta <0){
        R_output = limitValue(a*front_dis+b, str_limit, -1*str_limit) - limitValue((c*abs(theta)+d), turn_limit,-1*turn_limit)*1.2+ break_;
        L_output = limitValue(a*front_dis+b, str_limit, -1*str_limit) + limitValue((c*abs(theta)+d), turn_limit,-1*turn_limit)*0.8+ break_;
    }else{
        R_output = limitValue(a*front_dis+b, str_limit, -1*str_limit);
        L_output = limitValue(a*front_dis+b, str_limit, -1*str_limit);
    }

    cout << "e : "<< theta << " front dis " << front_dis << endl;
    cout << "Power cal : "<< endl<<a*front_dis+b <<" +- "<< c*pow(abs(theta),1.5)+d <<endl<<break_ <<endl;
    // make order
    order.mode = 1;
    order.R = R_output;
    order.L = L_output;
    order.time = timer_T;
    order.smooth = smooth;
    pub_order.publish(order);
  }

private: //private으로 NodeHandle과 publisher, subscriber를 선언한다
  ros::NodeHandle nh; 
  ros::Timer timer;
  ros::Timer param_update;
  ros::Publisher pub_order;
  ros::Subscriber sub_dir;
  ros::Subscriber sub_vel;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_algo");     // Initializes Node Name
  DriveAlgo algo; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
  return 0;
}