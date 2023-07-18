#include "ros/ros.h"                            // ROS Default Header File
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "control_pkg/order.h" 
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
using namespace std;
class MotorController
{
private:

//Motor Prameter
  const int R_max = 1912;
  int R_F_min = 1650; //a //min value motor work forward 1595
  const int R_stop = 1540; //c
  int R_B_min = 1445; //c
  const int R_min = 1108;
  int R_F_limit = 1800; //need remapping 
  int R_B_limit = 1200; //need remapping

  const int L_max = 1888;
  int L_F_min = 1630; //b //min value motor work forward 1583
  const int L_stop = 1540;
  int L_B_min = 1471; 
  const int L_min = 1120;
  int L_F_limit = 1660; //need remapping //1680
  int L_B_limit = 1350; //need remapping

// flag 
  float count_origin = 0;
  float count_down = 0;
  bool time_flag = 0;
  int order_flag= 0;
  bool enable_yaw_detector = 0;
  bool smooth_flag = 0;
  bool distance_detector =0;

//heading : absol   /   yaw : relative
  float yaw_target;
  float heading_now;
  float heading_start;
  float heading_target;

//gps distance data
  float total_distance =0;
  float target_distance =0;
  float before_x = 0;
  float before_y = 0;

//wallflow
  float follow_dis_L=3;
  float follow_dis_R=3;
  float L_wall_dis = 0;
  float R_wall_dis =0;
  float start_wall_dis = 3;
//control_parameter -100~100
  int R_output;
  int L_output;
  int R_target_output;
  int L_target_output;

//Turn Parameter
  const int min_point_turn_catch = 3; //deg
  const int max_point_turn_catch = 10; //deg
  const float point_turn_catch = 0.1; //ratio
  float turn_stop_point = 0;

//Brake Prameter
  const int max_brake_power = 10;

  float timer_T = 0.1;
public:
  MotorController()
  {
    param_update = nh.createTimer(ros::Duration(1), &MotorController::updaterCallback, this);
    pub_order_flag =nh.advertise<std_msgs::Bool>("Order_stats", 1);
    pub_ = nh.advertise<std_msgs::Int32MultiArray>("Motor_run", 100);
    sub_ = nh.subscribe("/RPY", 1, &MotorController::RPYcallback, this);
    timer = nh.createTimer(ros::Duration(0.1), &MotorController::timerCallback, this);
    sub_order = nh.subscribe("/run_order", 1, &MotorController::oderCallback, this);
    motor_pub = nh.advertise<std_msgs::Int32MultiArray>("Motor_run", 100); // [R, L]
    gps_sub = nh.subscribe("/gps_pose", 1, &MotorController::gpsCallback, this);
    sub_left = nh.subscribe("left_wall_dis", 1, &MotorController::leftWallCallback, this);
    sub_right = nh.subscribe("right_wall_dis", 1, &MotorController::rightWallCallback, this);
    sub_start = nh.subscribe("start_wall_dis", 1, &MotorController::startWallCallback, this);

    nh.getParam("/Motor_param/R_F_min", R_F_min);
    nh.getParam("/Motor_param/R_B_min", R_B_min);
    nh.getParam("/Motor_param/R_F_limit", R_F_limit);
    nh.getParam("/Motor_param/R_B_limit", R_B_limit);

    nh.getParam("/Motor_param/L_F_min", L_F_min);
    nh.getParam("/Motor_param/L_B_min", L_B_min);
    nh.getParam("/Motor_param/L_F_limit", L_F_limit);
    nh.getParam("/Motor_param/L_B_limit", L_B_limit);
  }
   
  /*
  float limitValue(float input, float max, float min);
  int limitValue(int input, int max, int min);
  float setDegree(float input);*/

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
  void upLinePrompt(int count)
  {
      for (int i = 0; i < count; ++i) {
          //printf("%c[2K",27);
          //cout<<"\33[2K"; //line clear
          //cout<<"\x1b[A"; //up line (ESC [ A) must be support VT100 escape seq
      }
  }
  int map(int input, int input_min, int input_max, int output_min, int output_max){
    if(!input){
      return 0;
    }else{
      return (input-input_min)*(output_max-output_min)/(input_max-input_min)+output_min;        
    }
  }
  int remapPower_R(int input_power){
    
    if(input_power>0){
      return map(input_power, 1, 100, R_F_min, R_F_limit);
    }else if (input_power<0){
      return map(input_power, -100, -1, R_B_limit, R_B_min);
    }else{
      return R_stop;
    }
  }
  int remapPower_L(int input_power){
    int output;
    if(input_power>0){
      return map(input_power, 1, 100, L_F_min, L_F_limit);
    }else if(input_power<0){
      return map(input_power, -100, -1, L_B_limit, L_B_min);
    }else{
      return L_stop;
    }

  }
  void leftWallCallback(const std_msgs::Float32::ConstPtr& input){
    L_wall_dis = input->data;
  }
  void rightWallCallback(const std_msgs::Float32::ConstPtr& input){
    R_wall_dis = input->data;
  }
  void startWallCallback(const std_msgs::Float32::ConstPtr& input){
    start_wall_dis = input->data;
  }
  void timerCallback(const ros::TimerEvent& event){
    
    std_msgs::Int32MultiArray Motor_run;
    Motor_run.data.push_back(remapPower_R(R_output));
    Motor_run.data.push_back(remapPower_L(L_output));
    motor_pub.publish(Motor_run);

    if(order_flag ==1){
      cout<< "==== Motor Pub "<<remapPower_R(R_output)<< " / " << remapPower_L(L_output)<< "===="<<endl;
      if(count_down>0){
        count_down = count_down - timer_T;
        time_flag = 1;
        if(enable_yaw_detector){
          if(yaw_target>0){ // left turn
            if(R_output<L_output)ROS_WARN("Power State Wrong with yaw = left turn\n But Output R: %d, L: %d", R_output, L_output);
            if(heading_now>turn_stop_point){ // fisnish angle order
              time_flag = 0;
              order_flag = 0;
              enable_yaw_detector =0;
              distance_detector = 0;
              R_output = 0;
              L_output = 0;
              smooth_flag = 0;
              ROS_WARN("SUCCESS");
              std_msgs::Bool orderstate;
              orderstate.data = 0;
              pub_order_flag.publish(orderstate);
              return;
            }
          }else{//right turn
            if(R_output>L_output)ROS_WARN("Power State Wrong with yaw = right turn\n But Output R: %d, L: %d", R_output, L_output);
            if(heading_now<turn_stop_point){ // fisnish angle order
              time_flag = 0;
              order_flag = 0;
              enable_yaw_detector =0;
              distance_detector = 0;
              R_output = 0;
              L_output = 0;
              ROS_WARN("YAW SUCCESS");
              //publish order end topic
              std_msgs::Bool orderstate;
              orderstate.data = 0;
              pub_order_flag.publish(orderstate);
              return;
            }
          }
        }
        if(distance_detector){
          if(total_distance>target_distance){
            ROS_WARN("DISTANCE SUCCESS ");
              time_flag = 0;
              order_flag = 0;
              enable_yaw_detector =0;
              distance_detector = 0;
              R_output = 0;
              L_output = 0;
              total_distance = 0;
              //publish order end topic
              std_msgs::Bool orderstate;
              orderstate.data = 0;
              pub_order_flag.publish(orderstate);            
          }
          
        }
        if(smooth_flag){ //smooth acc
          R_output += ((R_target_output-R_output)*timer_T/2);
          L_output += (L_target_output-L_output)*timer_T/2;
        }
        if(R_target_output < R_output){
          smooth_flag = 0;
        }
      //Not yet success
      cout<< count_down << "(s)   /   " << setDegree(heading_target-heading_now)<< "(deg) / "<< total_distance << "(m)"<<endl;
      upLinePrompt(2);
      //ROS_WARN("Time IN(%f)", count_down);
      

      }else{ // Time end
        ROS_WARN("Time OUT(%f)", count_down);
        order_flag = 0;
        time_flag = 0;
        R_output = 0;
        L_output = 0;
        total_distance = 0;
        
        std_msgs::Bool orderstate;
        orderstate.data = 0;
        pub_order_flag.publish(orderstate);
      }
    }
    if(order_flag ==5 || order_flag ==6){ // wall flow mode
      if(count_down>0){
        count_down = count_down - timer_T;
        if(order_flag == 5){ //right_wall
          if(R_wall_dis > follow_dis_R){
            R_output = 30;
            L_output = 70;
          }else{
            R_output = 70;
            L_output = 30;
          }
        }else{// left_wall
          if(L_wall_dis > follow_dis_L){
            R_output = 70;
            L_output = 30;
          }else{
            R_output = 30;
            L_output = 70;
          }
        }
      cout<< "==== real Pub "<<remapPower_R(R_output)<< " / " << remapPower_L(L_output)<< "===="<<endl;
      cout << "Ldis "<< L_wall_dis << " / " << follow_dis_L << endl;
      cout << "power : " << R_output<< " / " << L_output << endl;

      }else{ // Time end
        ROS_WARN("Time OUT(%f)", count_down);
        order_flag = 0;
        time_flag = 0;
        R_output = 0;
        L_output = 0;
        total_distance = 0;
        
        std_msgs::Bool orderstate;
        orderstate.data = 0;
        pub_order_flag.publish(orderstate);
      }
    }
  }
  
  void RPYcallback(const std_msgs::Float32MultiArray::ConstPtr& input){
    heading_now = input->data[2]; // update sensor yaw value
  }
  void updaterCallback(const ros::TimerEvent& event){
    // update parameter!
    nh.getParam("/Motor_param/R_F_min", R_F_min);
    nh.getParam("/Motor_param/R_B_min", R_B_min);
    nh.getParam("/Motor_param/R_F_limit", R_F_limit);
    nh.getParam("/Motor_param/R_B_limit", R_B_limit);

    nh.getParam("/Motor_param/L_F_min", L_F_min);
    nh.getParam("/Motor_param/L_B_min", L_B_min);
    nh.getParam("/Motor_param/L_F_limit", L_F_limit);
    nh.getParam("/Motor_param/L_B_limit", L_B_limit);
  }
  void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& input){
    float deltax = input->pose.position.x- before_x;
    float deltay = input->pose.position.y - before_y;
    float delta = sqrt(pow(deltax,2)+pow(deltay,2));
    //cout << "gps distance / x :" <<deltax<< " y : "<< deltay << " distance : " << delta <<endl;
    total_distance += delta;
    before_x = input->pose.position.x;
    before_y = input->pose.position.y;
  }
  void oderCallback(const control_pkg::order::ConstPtr& order){
    cout<< "input order mode : " << order->mode << endl;
    if(order->mode == 1){ // time_drive_mode
      order_flag = 1;
      cout<< "TIME_DRIVE_MODE : " << order->time<<"s " << order->R<<"/"<<order->L << "  SMOOTH : "<< order->smooth <<endl;
      count_down = order->time;
      if(order->smooth){
        R_target_output = order->R;
        L_target_output = order->L;
        smooth_flag = 1;
      }else{
        R_output = order->R;
        L_output = order->L;
        smooth_flag = 0;
      }
      
    }else if(order->mode ==2){// yaw_drive_mode
      order_flag = 1;
      cout<< "YAW_DRIVE_MODE : " << order->time<<"s " << order->R<<"/"<<order->L << " yaw: "<<order->yaw<<  "  SMOOTH : "<< order->smooth  <<endl;
      count_down = order->time;
      if(order->smooth){
        R_target_output = order->R;
        L_target_output = order->L;
        smooth_flag = 1;
      }else{
        R_output = order->R;
        L_output = order->L;
        smooth_flag = 0;
      }
      heading_start = heading_now;
      yaw_target = order->yaw;

      heading_target = heading_start + yaw_target; // +: left turn
      enable_yaw_detector = 1;
      heading_target = setDegree(heading_target);
      cout<< "stop margin" << limitValue(yaw_target*point_turn_catch, max_point_turn_catch, min_point_turn_catch)*yaw_target/abs(yaw_target) << endl;
      turn_stop_point = heading_target - limitValue(yaw_target*point_turn_catch, max_point_turn_catch, min_point_turn_catch)*yaw_target/abs(yaw_target);
      turn_stop_point = setDegree(turn_stop_point);
      cout<< heading_start << "deg  ->  "<<heading_target<< "deg    catch point = "<< turn_stop_point<< endl;

    }else if(order->mode ==3){// point_drive_mode
      order_flag = 1;
      cout<< "POINT_TURN_MODE : " << order->time<<"s " << order->R<<"/"<<order->L << order->yaw <<endl;
      count_down = order->time;
      R_output = order->R;
      L_output = order->L;
      heading_start = heading_now;
      yaw_target = order->yaw;
      smooth_flag = 0;

      heading_target = heading_start + yaw_target; // +: left turn
      enable_yaw_detector = 1;
      heading_target = setDegree(heading_target);
      cout<< "margin" << limitValue(yaw_target*point_turn_catch, max_point_turn_catch, min_point_turn_catch)*yaw_target/abs(yaw_target) << endl;
      turn_stop_point = heading_target - limitValue(yaw_target*point_turn_catch, max_point_turn_catch, min_point_turn_catch)*yaw_target/abs(yaw_target);
      turn_stop_point = setDegree(turn_stop_point);
      cout<< heading_start << "deg  ->  "<<heading_target<< "deg    catch point = "<< turn_stop_point<< endl;
    }else if(order->mode ==4){// brake_drive_mode
      order_flag = 1;
      smooth_flag = 0;
      count_down = order->time;
      cout<< "BRAKE_MODE : " << order->time<<"s "<<endl;
      count_down = order->time;  
      R_output = -limitValue(R_output, -max_brake_power, +max_brake_power);
      L_output = -limitValue(L_output, -max_brake_power, +max_brake_power);
    }else if(order->mode ==5){ //distance_drive_mode
      order_flag = order->mode;
      ROS_WARN("================= %d", order_flag);
      //order_flag = 5;
      order_flag = 5;
      cout<< "Wallfollow_MODE : " << order->time<<"s " << order->R<<"/"<<order->L << "  DIS:  "<< order->dis << "  SMOOTH : "<< order->smooth << order_flag <<endl;
      count_down = order->time;
      follow_dis_R = order->dis;
      order_flag = 5;
      
      smooth_flag = 0;
      
    }else if (order->mode ==6){
      order_flag = 6;
      cout<< "Wallfollow_MODE : " << order->time<<"s " << order->R<<"/"<<order->L << "  DIS:  "<< order->dis << "  SMOOTH : "<< order->smooth << order_flag <<endl;
      count_down = order->time;
      order_flag = 6;
      follow_dis_L = order->dis;
    }else{
      ROS_WARN("WRONG MODE INPUT: %d\n mode 1: time_drive \n mode 2: yaw_drive \n mode 3: point_turn \n mode 4: brake mode", order->mode);
    }
  }

  
  /*
  void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
  {
    PUBLISHED_MESSAGE_TYPE output;
    //callback 함수에서 받은 input을 사용해서 output을 만들고 이를 pub한다.
    pub_.publish(output);
  }*/

private: //private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle nh; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber sub_order;
  ros::Subscriber gps_sub;
  ros::Timer timer;
  ros::Publisher motor_pub;
  ros::Publisher pub_order_flag;
  ros::Timer param_update;
  ros::Subscriber sub_left;
  ros::Subscriber sub_right;
  ros::Subscriber sub_start;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");     // Initializes Node Name
  MotorController Controller; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
  return 0;
}

/*

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "motor_controller");     // Initializes Node Name
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


*/