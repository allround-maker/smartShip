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
  bool docking = 0;


//gps distance data
  float total_distance =0;
  float target_distance =0;
  float before_x = 0;
  float before_y = 0;


//control_parameter -100~100
  int R_output = 0;
  int L_output = 0;

// Limit Value : 
  int str_limit = 25;
  int turn_limit = 70;
  int break_limit = -50;
//Power Parameter
// R_ output = a
  float a=10;
  float b=1;
  float c=0.7;
  float d=8;

// break_value
  float break_distance = 1;
  float velocity;
  float FPtolidar = 0.4;
  float brake_value = -50;
// Sensor Data
  float front_dis;
  float theta;
  float theta_dis;
  float front_dis_for_break;

// Order T
  float timer_T = 0.2;
  bool smooth=0;
  float timeout_timer=0;;

public: 
  DriveAlgo()
  {
    pub_order = nh.advertise<control_pkg::order>("/run_order", 1);
    sub_dir = nh.subscribe("/Direction_for_avoid", 1, &DriveAlgo::dirCallback, this);
    param_update = nh.createTimer(ros::Duration(timer_T), &DriveAlgo::updaterCallback, this);
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
  float map(float x,float in_min, float in_max,float out_min, float out_max){  
    return(x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
  }
  
  void dirCallback(const std_msgs::Float32MultiArray::ConstPtr& input){
    //update data
    theta = input->data[0];
    theta_dis = input->data[1];
    front_dis = input->data[2];
    front_dis_for_break = input->data[3];
    timeout_timer = 100;
    
  }
  void updaterCallback(const ros::TimerEvent& event){
    // update parameter!
    nh.getParam("/control_param/a", a);
    nh.getParam("/control_param/b", b);
    nh.getParam("/control_param/c", c);
    nh.getParam("/control_param/d", d);
    nh.getParam("/control_param/smooth", smooth);
    nh.getParam("/control_param/docking_mode", docking); //ver 3 update
  }

  void veloCallback(const geometry_msgs::Vector3Stamped::ConstPtr& input){
    velocity = input->vector.x;
  }
  void timerCallback(const ros::TimerEvent& event){
    // It's order time!
    int break_ = 0;
    control_pkg::order order;
    if(timeout_timer <0 ){
      ROS_WARN("Timeout dir call back doen't work %d", timeout_timer);
      order.mode = 1;
      order.R = 0;
      order.L = 0;
      order.time = timer_T;
      order.smooth = smooth;
      pub_order.publish(order);
      return;
    }else{
      timeout_timer = timeout_timer -1;
    }


    if((front_dis_for_break<break_distance)&&!docking){
      break_ = (break_distance-front_dis_for_break)*100/(break_distance-FPtolidar)*-1;
      break_ = limitValue(break_, 0, brake_value); 
      //break_ = 0;
    }
      cout << "dir       :" <<theta << endl;
      cout << "front     : "<<front_dis << endl;
      cout << "break_dis :" <<front_dis_for_break << endl;

    
    float str_output = limitValue(a*front_dis+b, str_limit, -str_limit);
    if(front_dis < 2) str_output = 0;
    float turn_output  = limitValue((c*abs(theta)+d), turn_limit,-turn_limit);
    float reverse_thrust = limitValue(map(front_dis, 1.5,3, 0.2,0), 0.2, 0);
    cout << "str_output : "<< str_output <<endl;
    cout << "turn_output: "<< turn_output<< endl;
    cout << "reverse_thr: "<< reverse_thrust<< endl;  


    if(theta>50){
      R_output = turn_output*0.8;
      L_output = -turn_output*1.2;
      ROS_WARN("fall angle -point_TURN -Right");
    }else if(theta > 0){
      //R_output = str_output + turn_output*(1.0-reverse_thrust)+ break_;
      R_output = str_output + turn_output*(0.8)+ break_;
      //L_output = str_output - turn_output*(1.0+reverse_thrust)+break_;
      L_output = str_output - turn_output*(1.2)+break_;
    }else if(theta<-50){
      R_output = -turn_output*(1.0+reverse_thrust);
      L_output = turn_output*(1.0-reverse_thrust);
      ROS_WARN("fall angle -point_TURN -Left");
    }else if(theta <=0){
        //R_output = str_output - turn_output*(1.0+reverse_thrust)+ break_;
        R_output = str_output - turn_output*(1.2)+ break_;
        //L_output = str_output + turn_output*(1.0-reverse_thrust)+ break_;
        L_output = str_output + turn_output*(0.8)+ break_;
    }else{
        R_output = -turn_output*1.2;
        L_output = turn_output*0.8;
    }

    cout << "=="<<R_output << " / " <<  L_output<<"=="<< endl;
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
  ros::init(argc, argv, "drive_algo3");     // Initializes Node Name
  DriveAlgo algo; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
  return 0;
}