#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <limits.h>
#include <cmath>
#include <float.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

using namespace std;
#define PI 3.14159
#define inf_to 8
float Breath = 0.6;
float Breath_real = 0.3;
float side_dir = 20; //-79 
float side_dir2=35;
bool docking = 0;

vector<float> before_dir = {0};
void printVec(vector<float> input){
    for(auto i: input){
        cout<< i <<", " ;
    }
    cout<<endl;
}
void printVec(vector<bool> input){
    for(auto i: input){
        cout<< i <<", " ;
    }
    cout<<endl;
}
void printVec(vector<int> input){
    for(auto i: input){
        cout<< i <<", " ;
    }
    cout<<endl;
}
float setDegree(float input){
    if(input>180) return input-360;
    if(input<-180) return input+360;
    return input;
  }

class AxisScan{
    private:
    vector<vector<float>> scan;
    vector<float> origin_scan;
    vector<float>::iterator os_iter = origin_scan.begin();
    float indexs_angle = 0.2649006694;
    vector<float> collision_dis;
    vector<float> closeToTarget;
    vector<float> test_Vec;
    //float indexs_angle = 360/1360;
    private:

    public:
    AxisScan(){
    }

    void updateScan(vector<float> rescan){
        cout<< "update" << endl;
        origin_scan.clear();
        

        for(int i=0; i<rescan.size(); i++){
            if((1210<i<1325)||(35<i<50)){
                origin_scan.push_back(inf_to);
            }else{
                if(rescan[i]==0)
                    origin_scan.push_back(inf_to);
                else{
                    origin_scan.push_back(rescan[i]);
                }
            }
        }    
        
        
        scan.clear();
        for(int i=0; i<rescan.size(); i++){
            vector<float> point;
            float angle = (i*indexs_angle-180.0)*PI/180;
            
            if(!rescan[i]){
                point.push_back(rescan[i]*cos( angle ));
                point.push_back(rescan[i]*sin( angle ));
            }else{
                point.push_back((float)inf_to*cos( angle ));
                point.push_back((float)inf_to*sin( angle ));
            }
            scan.push_back(point);  
            
        }

    }

    float distanceToCollision(float dir, float B){ // calculate collision distance at all data
        // y= ax+b
        float e = dir/180.0*PI;
        float a = tan(e);
        float k = sqrt(a*a+1);
        float scan_deg = 55.0;
        vector<float> dis;
        int start_scan = (int)((dir-scan_deg+180)/indexs_angle);
        int end_scan = (int)((dir+scan_deg+180)/indexs_angle);
        
        
        for(int i=start_scan; i<end_scan; i++){
            int j = i;
            if(j<0) j= j+scan.size();
            if(j>=scan.size()) j=j-scan.size();
            float dis_with_line = abs(a*scan[j][0]-scan[j][1])/k;

            if( dis_with_line< B){
                float dis_ = origin_scan[j];
                if(dis_ == 0 ) dis_ = inf_to;
                if(dis_>inf_to) dis_ = inf_to;
                dis.push_back((dis_*cos((dir-((float)j*indexs_angle+180.0))/180*PI)));
                if(dis_with_line==0){
                    //cout << "collision point : "<< j*indexs_angle-180<<"(deg) / " <<dir<<"(deg) "<< scan[j][0]<<", "<<scan[j][1]<< "dis with line " << dis_with_line << endl;
                    //cout << "/"<<a<<"/"<<scan[j][0]<<"/"<<scan[j][1]<<"/"<<k<<endl;

                    //cout << "distance : "<< origin_scan[j] << "     theta : " << dir-((float)j*indexs_angle+180.0) << endl;
                    //cout << dir << "    /   " << ((float)j*indexs_angle-180.0) << endl;
                }
                //cout << "======================dis "<< dir << "=======================" << endl;
                //printVec(dis);
            }
        }
        return *min_element(dis.begin(), dis.end());
    }

    float scanCollision(float target_yaw){
        collision_dis.clear();
        closeToTarget.clear();
        for(int deg = -180; deg<180; deg=deg+1){
            float collision_data = distanceToCollision(deg, Breath);
            collision_dis.push_back(collision_data); // update collision_dis

            cout << "degree = "<< deg <<"   distance : "<<collision_data <<endl;
            //cout << "close to target "<< collision_data * cos(deg/180.0*PI) << endl;
            /*
            if(collision_data > 3){
                closeToTarget.push_back(2*cos(((float)deg-target_yaw)/180.0*PI));
            }else{
                closeToTarget.push_back(collision_data * cos(((float)deg-target_yaw)/180.0*PI) );
            }*/
            float edit_theta = 1;
            if(distanceToCollision(0, Breath)>3){
                edit_theta = (100.0-(float)abs(deg)/10)/100;
                //cout << "edit_theta = " << edit_theta << endl;
            }
            

            float center_close = collision_data * cos(((float)deg+target_yaw)/180.0*PI);
            float left_close = collision_data * cos(((float)deg+(target_yaw+side_dir))/180.0*PI);
            float right_close = collision_data * cos(((float)deg+(target_yaw-side_dir))/180.0*PI);
            float left_close2 = collision_data * cos(((float)deg+(target_yaw-side_dir2))/180.0*PI);
            float right_close2 = collision_data * cos(((float)deg+(target_yaw+side_dir2))/180.0*PI);
            if(left_close <0) left_close=0;
            if(right_close<0) right_close=0;
            if(!docking){
                if(collision_data>2.2){ //min distance for chosse
                    //closeToTarget.push_back((left_close + right_close));
                    //closeToTarget.push_back(center_close);
                    //closeToTarget.push_back((left_close + right_close)*edit_theta);
                    vector<float> score;
                    score.push_back(center_close*1.2);
                    //cout<<deg << " deg : "<<collision_data<<"(m)    " <<  "target_yaw : " << target_yaw << "    cos (deg)" << ((float)deg+(target_yaw)) <<  endl;
                    score.push_back(left_close);
                    score.push_back(right_close);
                    //score.push_back(left_close2);
                    //score.push_back(right_close2);

                    closeToTarget.push_back(*max_element(score.begin(), score.end()));
                }else{
                    closeToTarget.push_back(0);
                }
            }else{
                closeToTarget.push_back((left_close + right_close));
            }


        }
        cout << "=============================== collision===================" << endl; ;
        printVec(collision_dis);
        for(int i = 0; i<collision_dis.size(); i++){
            //cout << "deg : "<< i << "   collison dis : "<< collision_dis[i] << "closeToTarget : "<<closeToTarget[i] << endl;         
        }
        for(int i = 0; i< collision_dis.size(); i++){
            //closeToTarget.push_back( collision_dis[i]*cos(((deg-heading_now)-(i-180))/180*PI) );
            //closeToTarget.push_back( collision_dis[i]*cos((float)(i*1.0-180.0)/180*PI) );
        }
        int max_index = max_element(closeToTarget.begin(), closeToTarget.end()) - closeToTarget.begin();
        cout << "final dir : " << (float)max_index*1.0-180.0 << endl;
        cout << "================== close to target =====================" << endl;
        printVec(closeToTarget);
        cout << closeToTarget[max_index] << "   deg: "<< -max_index+180<< " size : " << closeToTarget.size() << endl;

        return (float)max_index*(1.0)-180.0;
    }

    float findMin(float finish){
        int start_index = (int)((180.0)/indexs_angle);
        int finish_index = (int)((setDegree(finish)+180.0)/indexs_angle);
        //cout<< start_index <<"  ///  " << finish_index << "  ///   " <<origin_scan.size() << endl;
        
        if(start_index>finish_index){
            return *min_element(origin_scan.begin()+finish_index, origin_scan.begin()+start_index);
            
        }else{
            return *min_element(origin_scan.begin()+start_index, origin_scan.begin()+finish_index);
        }
        
    }
};



class Planner
{
private:
    float heading_now = 0;
    AxisScan axis_scan;
    float target_heading = 110; // -42
    float timer_T = 0.2;
    vector<float> test_Vec;
    
    bool enalbe_planner = 1;

public:
  Planner()
    {
        sub_RPY = n_.subscribe("/RPY", 1, &Planner::RPYcallback, this);
        sub_ = n_.subscribe("/scan", 1, &Planner::scanCallback, this);
        pub_ = n_.advertise<std_msgs::Float32MultiArray>("/Direction_for_avoid", 1000);
        timer = n_.createTimer(ros::Duration(timer_T), &Planner::timerCallback, this);

        sub_target = n_.subscribe("/target_heading", 1, &Planner::updateTargetHeading, this);
        sub_enalbe = n_.subscribe("/enable/planner", 1, &Planner::updateEnablePlanner, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        /*
        std_msgs::Float32MultiArray dis_arr; //origin
        dis_arr.data = scan->ranges;
        std_msgs::Float32 Disting;
        
        cout << "input_size" <<dis_arr.data.size()<<endl;
        */
        axis_scan.updateScan(scan->ranges);
        /* erase frame !!!!!!
        
        for(int i=0; i<scan->ranges.size(); i++){
            cout << " scan : " << scan->ranges[i] << "  i = "<< i<< endl;
        }*/
        
        //cout << "vec size checek 1  " <<test_Vec.size() << endl;
        
        // input :target dir(relative) output : go dir
        float dir_ = axis_scan.scanCollision(target_heading-heading_now);
        float distance_ = axis_scan.distanceToCollision(dir_, Breath);
        //float foward_dis_ = axis_scan.distanceToCollision(0, Breath);
        float foward_dis_ = axis_scan.findMin(dir_);//edit 
        float foward_dis_for_break = axis_scan.distanceToCollision(0,Breath_real);
        
        cout << "dir : " << dir_<<endl << "   distance at dir : "<< distance_<< endl << "    distance at foward : "<< foward_dis_ <<endl<< "   target_yaw : " <<  target_heading-heading_now<< endl;
        std_msgs::Float32MultiArray output;
        output.data.push_back(dir_);
        output.data.push_back(distance_);
        output.data.push_back(foward_dis_);
        output.data.push_back(foward_dis_for_break);
        if(enalbe_planner){
            pub_.publish(output);
        }
        //(스캔값, 기준방향, 가시거리)
    }
    void RPYcallback(const std_msgs::Float32MultiArray::ConstPtr& input){
        heading_now = input->data[2]; // update sensor yaw value
    }

    void timerCallback(const ros::TimerEvent& event){

    }
    void updateTargetHeading(const std_msgs::Float32::ConstPtr& heading){
        target_heading = setDegree(heading->data);
    }

    void updateEnablePlanner(const std_msgs::Bool::ConstPtr& input){
        enalbe_planner = input->data;
    }
private: //private으로 NodeHandle과 publisher, subscriber를 선언한다
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Subscriber sub_RPY;
    ros::Publisher pub_;
    ros::Timer timer;
    ros::Subscriber sub_target;
    ros::Subscriber sub_enalbe;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner2");     // Initializes Node Name
  Planner pathPlan; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
  return 0;
}