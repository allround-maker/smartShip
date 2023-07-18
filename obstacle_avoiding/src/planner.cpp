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

using namespace std;
#define PI 3.14159
#define inf_to 8
float Breath = 0.6;
float Breath_real = 0.3; 
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
    float indexs_angle = 0.2649006694;
    vector<float> collision_dis;
    vector<float> closeToTarget;

    //float indexs_angle = 360/1360;
    private:

    public:
    AxisScan(){
    }

    void updateScan(vector<float> rescan){
        cout<< "update" << endl;
        origin_scan.clear();
        origin_scan = rescan;
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
                dis.push_back((dis_*cos((dir-((float)j*indexs_angle+180.0))/180*PI)));
                if(dis_with_line==0){
                    //cout << "collision point : "<< j*indexs_angle-180<<"(deg) / " <<dir<<"(deg) "<< scan[j][0]<<", "<<scan[j][1]<< "dis with line " << dis_with_line << endl;
                    //cout << "/"<<a<<"/"<<scan[j][0]<<"/"<<scan[j][1]<<"/"<<k<<endl;

                    //cout << "distance : "<< origin_scan[j] << "     theta : " << dir-((float)j*indexs_angle+180.0) << endl;
                    //cout << dir << "    /   " << ((float)j*indexs_angle-180.0) << endl;
                }

            }
        }
        return *min_element(dis.begin(), dis.end());
    }

    float scanCollision(float target_yaw){
        collision_dis.clear();
        closeToTarget.clear();
        for(int deg = -180; deg<180; deg=deg+1){
            float collision_data = distanceToCollision(deg, Breath);
            collision_dis.push_back(distanceToCollision(deg, Breath)); // update collision_dis

            //cout << "degree = "<< deg <<"   distance : "<<collision_data <<endl;
            //cout << "close to target "<< collision_data * cos(deg/180.0*PI) << endl;
            /*
            if(collision_data > 3){
                closeToTarget.push_back(2*cos(((float)deg-target_yaw)/180.0*PI));
            }else{
                closeToTarget.push_back(collision_data * cos(((float)deg-target_yaw)/180.0*PI) );
            }*/
            closeToTarget.push_back(collision_data * cos(((float)deg-target_yaw)/180.0*PI) );

        }
        //printVec(collision_dis);
        for(int i = 0; i<collision_dis.size(); i++){
            //cout << "deg : "<< i << "   collison dis : "<< collision_dis[i] << "closeToTarget : "<<closeToTarget[i] << endl;         
        }
        for(int i = 0; i< collision_dis.size(); i++){
            //closeToTarget.push_back( collision_dis[i]*cos(((deg-heading_now)-(i-180))/180*PI) );
            //closeToTarget.push_back( collision_dis[i]*cos((float)(i*1.0-180.0)/180*PI) );
        }
        int max_index = max_element(closeToTarget.begin(), closeToTarget.end()) - closeToTarget.begin();
        cout << "final dir : " << (float)max_index*(-180.0) << endl;
        return (float)max_index*(1.0)-180.0;
    }
};



class Planner
{
private:
    float heading_now = 0;
    AxisScan axis_scan;
    float target_heading = 110; // -42
    vector<float> test_Vec;
    

public:
  Planner()
    {
        sub_RPY = n_.subscribe("/RPY", 1, &Planner::RPYcallback, this);
        sub_ = n_.subscribe("/scan", 1, &Planner::scanCallback, this);
        pub_ = n_.advertise<std_msgs::Float32MultiArray>("/Direction_for_avoid", 1000);
        timer = n_.createTimer(ros::Duration(1), &Planner::timerCallback, this);

        for(int i=0; i<1360; i++){
            test_Vec.push_back(inf_to);
        }
        
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
        //axis_scan.updateScan(test_Vec);
        //cout << "vec size checek 1  " <<test_Vec.size() << endl;
        
        // input :target dir(relative) output : go dir
        float dir_ = axis_scan.scanCollision(target_heading-heading_now);
        float distance_ = axis_scan.distanceToCollision(dir_, Breath);
        float foward_dis_ = axis_scan.distanceToCollision(0, Breath);
        float foward_dis_for_break = axis_scan.distanceToCollision(0,Breath_real);
        cout << "dir : " << dir_ << "   distance at dir : "<< distance_ << "    distance at foward : "<< foward_dis_ << "   target_yaw : " <<  target_heading-heading_now<< endl;
        std_msgs::Float32MultiArray output;
        output.data.push_back(dir_);
        output.data.push_back(distance_);
        output.data.push_back(foward_dis_);
        output.data.push_back(foward_dis_for_break);
        pub_.publish(output);
        //(스캔값, 기준방향, 가시거리)
    }
    void RPYcallback(const std_msgs::Float32MultiArray::ConstPtr& input){
        heading_now = input->data[2]; // update sensor yaw value
    }

    void timerCallback(const ros::TimerEvent& event){

    }
    void updateTargetHeading(float heading){
        target_heading = setDegree(heading);
    }

private: //private으로 NodeHandle과 publisher, subscriber를 선언한다
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Subscriber sub_RPY;
    ros::Publisher pub_;
    ros::Timer timer;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");     // Initializes Node Name
  Planner pathPlan; //클래스 객체 선을 하게 되면 모든게 된다.
  ros::spin();
  return 0;
}