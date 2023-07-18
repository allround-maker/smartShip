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
class RidarDir
{
    private:
        ros::NodeHandle n_;
        ros::Subscriber sub_;
        ros::Subscriber sub_RPY;
        ros::Publisher pub_;
        //defult 
        float ST_DR = 0;
        float VS = 2;
        float Theta = 85; //angle to figure out forword distance
        float B = 0.6; // half breath of vessel
        float alpa = 0.1; //margin for breath of vessel
    private:
        float heading_now = 0;
    private:
        float StraightRange(vector<float> Lidar_data)
        {	
            vector<float> range_array;
            int middle_index = 679;
            int range_start = Theta / 0.2649006694;
            for(int i = middle_index - range_start; i < middle_index + range_start; i++)
            {
                range_array.push_back(Lidar_data[i]);
            }
            vector<float> StraightRange;
            for(int i = 0; i < range_array.size(); i++)
            {   
                if(range_array[i] == 0){
                    StraightRange.push_back(10);
                }
                else if(B/2 + alpa > range_array[i] * abs(sin( (i - range_array.size()/2) *Theta*2/range_array.size() * 3.1416 / 180) ))
                {
                    StraightRange.push_back(range_array[i]*cos((i - range_array.size()/2) *Theta*2/range_array.size() * 3.1416 / 180));
                }
                else
                {
                    //ROS_WARN("FUCK U");
                }
                //cout << B + alpa << " > " << range_array[i] <<"*" <<  " sin( " << (i - range_array.size()/2)  << "*"<< 360/range_array.size() << endl;
            }
            float forword_dst = *min_element(StraightRange.begin(), StraightRange.end());                                          
            return forword_dst;
        }
        vector<float> Change_inf(vector<float> Lidar_data)
        {
            vector<float> No_inf;
            for(int i = 0; i<Lidar_data.size();i++)
            {
                if(Lidar_data[i] == 0.0)
                {
                    No_inf.push_back(0.0);
                }
                else
                {
                    No_inf.push_back(Lidar_data[i]);
                }
            }
            return No_inf;
        }
        vector<float> GiveMargin(vector<float> Lidar_data, float margin_L)
        {
            vector<float> after_margin = Lidar_data;
            //float margin_L = 0.05;
            float bf_data = 0;
            float distance_obj;
            float indexs_angle = 0.2649006694;
            int index_margin;
            for(int i = 0; i < Lidar_data.size(); i++)
            {
                if(bf_data == 0 && Lidar_data[i]>=0.1 )
                {
					distance_obj = Lidar_data[i];
                    float angle_margin_L = (atan(margin_L/distance_obj)* 180 / 3.1416);
                    float index_marginx_L = (angle_margin_L/indexs_angle);
                    int index_margin_L = (int)index_marginx_L;
                    //cout<< distance_obj << ", "<<distance_obj/margin_L<<", "<<angle_margin_L << "deg, "<< index_margin_L<<"ea"<<endl;
                    for(int add=1; add <= index_margin_L; add++)
                        {
                            if((i-add)>=0){
                                after_margin[i-add]=Lidar_data[i];
                            }
                        }        
                }
                bf_data =Lidar_data[i];
            }
            for (int i=Lidar_data.size(); i>0; i--)
            {
                if(bf_data == 0 && Lidar_data[i]>=0.1)
                {
                    distance_obj = Lidar_data[i];
                    float angle_margin = (atan(margin_L/distance_obj)* 180 / 3.1416);
                    float index_marginx_R = (angle_margin/indexs_angle);
                    int index_margin_R = (int)index_marginx_R;
                        for(int add=1; add <= index_margin_R; add++)
                    {
                            //cout<<index_margin_R<<endl;
                        if((i+add) < Lidar_data.size())
                            {
                                //Lidar_data[i+add]=Lidar_data[i];
                                //cout<< Lidar_data[i+add] <<"<-"<< Lidar_data[i]<< endl;
                                //cout<< "index" << i+add << ", " << i<< endl;
                                after_margin[i+add]=Lidar_data[i];
                               
 
                            }
                        }
                }
                bf_data = Lidar_data[i];
            }
            return after_margin;
        }
 
        vector<bool> Detecting(vector<float> after_margin, float MaDis)
        {
            vector<bool> zero__;
            for(int i = 0; i < after_margin.size(); i++)
            {
                if(after_margin[i] <= 0.001 || after_margin[i]>MaDis)
                {
                    zero__.push_back(1);
                }
                else
                {
                    zero__.push_back(0);
                }
            }
            return zero__;
        }
 
        /*vector<float> MakeMargin(vector<float> input_vec, int marginsize){
            vector<float> remove_index;
            int before_data = 0;
            for(int i=0; i<input_vec.size(); i++){
                if(before_data == 0 && input_vec[i]>=0.0001 ){
                    for(int add=0; add < marginsize; add ++){
                        remove_index.push_back(i+add);
                    }
                }
                before_data = input_vec[i];
            }
            for(int i=input_vec.size(); i>0; i--){
                if(before_data == 0 && input_vec[i]>=0.0001 ){
                    for(int add=0; add < marginsize; add++){
                        if(i-add>0){
                            remove_index.push_back(i-add);
                        }
                    }
                }
                before_data = input_vec[i];
            }
            for(auto i: remove_index){
                input_vec[i]=0;
            }
            return input_vec;
        }*/
 
        vector<int> Area_calculation(vector<bool> savezone)
        {
            vector<int> sum;
            sum.push_back (savezone[0]);
            for(int i = 1; i < savezone.size(); i++)
            {
                if(savezone[i] == 1)
                {
                    sum.push_back (sum[i-1] + 1);
                }
                else
                {
                    sum.push_back (0);
                }
            }
            return sum;
        }
 
        /*vector<int> MakeGroupArray(vector<float> zero__){
            vector<float> output_vec;
            vector<float> my_vec = zero__;
            my_vec.push_back(0);
            float before_data = 0;
            for(int i=0; i<my_vec.size(); i++)
            {
                if (my_vec[i]==0 && before_data!=0)
                {
                    output_vec[i-1]=before_data;
                    output_vec.push_back(0);
                }
                else if(my_vec[i]==1 && before_data==0)
                {
                    output_vec.push_back(1);
                }
                else
                {
                    output_vec.push_back(0);
                }
                before_data=my_vec[i];
            }
            output_vec.pop_back();
            return output_vec;
        }*/
       
 
        vector<float> Angle(vector<int> input_direction)
        {
            vector<float> direction_angle;
            float angle = 0;
            float a;
            float b;
            for(int i = 0; i < input_direction.size(); i++)
            {
                if(input_direction[i] == 0 );
                else
                {
                    a = 0.2649006694;
                    b = i;
                    if(i >= input_direction.size()/2)
                    {
                        b = i - input_direction.size()/2;
                    }
                    angle = a * b;
                    if(i < input_direction.size()/2)
                    {
                        angle = -1 * a*(input_direction.size()/2 - b);
                    }
                    direction_angle.push_back (angle);
                }
            }
            return direction_angle;
        }
 
        float findMinDegree(vector<float> direction_angle, float target_dir){
            cout << target_dir << endl;
            vector<float> nums;
            for (int i=0; i < direction_angle.size(); i++) {
              nums.push_back(abs(direction_angle[i] - (target_dir-heading_now)));
            }
            //cout<< "target_dir" << target_dir << endl;
            //cout <<"dir angle :"<< endl;
            //printVec(direction_angle);
            //cout<<"nums : " << endl;
            //printVec(nums);
            int min_index = min_element(nums.begin(), nums.end()) - nums.begin();
            //ROS_WARN("findal degree %f", direction_angle[min_index]);
            printVec(direction_angle);
            return direction_angle[min_index];
        }
 
 /*
        float Min_Degree(vector<int> nums, vector<float>DirectionAngle , float target_degree)
        {
            float tums;
            //float min = *min_element(nums.begin(), nums.end());
            float min_index = min_element(nums.begin(), nums.end()) - nums.begin();

            tums.push_back(DirectionAngle[min_index]);
            return tums[0];
        }
*/
        vector<float> FindIndex(float tums, vector<float> ukiss, float NumRangePoints) // tums : choose dirction
        {
            vector<float> bigi;
            float avv = tums;
            float a = 0.2649006694;
            float angle_indexx = (avv/ a + 180);
            int angle_index = (int)angle_indexx;
            float distance_above = ukiss[angle_index];
            for(int i =(angle_index - NumRangePoints); i < (angle_index+NumRangePoints); i++)
            {
                int x;
                if(i >= ukiss.size())
                {
                    x = i -ukiss.size();   
                }
                else if (i < 0)
                {
                    x = ukiss.size() + i;            
                }
                else 
                {
                    x=i;
                }
                bigi.push_back(ukiss[x]);
            
            }
            //cout <<"bigi size : "<< bigi.size() << endl;
            return bigi;
        }

        vector<float> Rebuild(vector<float> bigi)
        {

            
            sort(bigi.begin(), bigi.end());

            return bigi;
        }

        float AvgAboveDist(vector<float> ssml, int num_dlt_points)
        {
            float answer =0;
            float avg_dist;
            vector<float> xxml;
            for(int i = num_dlt_points; i < ssml.size() - num_dlt_points; i++)
            {
                //cout << "i : "<< i << endl;
                if(ssml[i] == 0.0)
                {
                    xxml.push_back(1.5);
                }
                else
                {
                    xxml.push_back(ssml[i]);
                }
                /*if(ssml[i] > 10000)
                {
                    ssml.push_back(25);
                }*/
                answer += xxml[i-num_dlt_points]/14;
                //cout<< "answer : " << answer << endl;
            }
            //cout<< "size ======================================================" << endl;
            //printVec(ssml);
            //printVec(xxml);
            //avg_dist = answer/(xxml.size()-num_dlt_points*2);
            //cout<<"xxml size : "<< xxml.size()<< endl;
            //cout << answer << endl; 
            //cout<< num_dlt_points<< endl;
            //return avg_dist;
            return answer;
        }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool DISTING(vector<bool> zero__)
        {
            bool a = 0;
            for(int i = 0; i < zero__.size(); i++)
            {
                if(zero__[i] == 0)
                    a = a;
                else
                    a += zero__[i];
            }
            return a;
        }
   
        vector<float> FindDir(vector<float> ukiss, float margin_L, float target_dir, float MaDis, int NumRangePoints, int num_dlt_points)
        {   
            float forword_distance = StraightRange(ukiss); //forword distance for propullsion power
            vector<float> no_inf = Change_inf(ukiss); // recive origin data /scan -> inf transform 0;
            //cout<<endl<<ukiss.size();
            vector<float> soull = GiveMargin(no_inf, margin_L); // size up object for margin
            vector<bool> myzero = Detecting(soull, MaDis); //input( can go : 1 , cant go 0);
            //printVec(soull);
            //printVec(myzero);
            if(DISTING(myzero)){
				//vector<float> input_vec = MakeMargin(myzero, 1);
				vector<int> mysum = Area_calculation(myzero);  // grouping
                //cout << " mysum "<< endl;
                //printVec(mysum);
				vector<float> direction_angle = Angle(mysum);
                //cout << "direction angle : "<< endl;
                //printVec(direction_angle);

				//vector<int> nums = AbsVec(direction_angle, target_dir); // input( all angle(cango)
                float min_Degree = findMinDegree(direction_angle, target_dir);
                //cout << "final_degree : "<< min_Degree;

                vector<float> mybigi = FindIndex(min_Degree, ukiss, NumRangePoints);
                vector<float> myssml = Rebuild(mybigi);
                float realdeal = AvgAboveDist(myssml, num_dlt_points);
                vector<float> Angles_Distance;
                Angles_Distance.push_back(min_Degree);
                Angles_Distance.push_back(realdeal);
                Angles_Distance.push_back(forword_distance);
                //NumRangePoints: 목표방향의 물체 탐지 범위 확장 index 값
                //num_dlt_points: 목표방향의 물체의 거리 평균 계산 전 제외할 index 값
                return Angles_Distance;
			}
            else
            {
				ROS_WARN ("No_data");
                return before_dir;
			}
        }
               
    public:
        RidarDir(){
            sub_RPY = n_.subscribe("/RPY", 1, &RidarDir::RPYcallback, this);
            
            sub_ = n_.subscribe("/scan", 10, &RidarDir::scanCallback, this);
            pub_ = n_.advertise<std_msgs::Float32MultiArray>("/Direction_for_avoid", 1000);
        }
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            std_msgs::Float32MultiArray dis_arr;
            dis_arr.data = scan->ranges;
            std_msgs::Float32 Disting;
            std_msgs::Float32MultiArray output;

            if(!n_.getParam("Standard_DR", ST_DR)){ROS_WARN("Fail to get standard_dir");}
            if(!n_.getParam("Visibility", VS)){ROS_WARN("Fail to get Visibility");}
            //Disting.data = Detecting(dis_arr.data, ST_DR, VS);
            output.data = FindDir(dis_arr.data, 0.1, ST_DR, VS, 10, 3);
            

            //output.data = FindDir(dis_arr.data, ST_DR, VS);
            pub_.publish(output);
            //(스캔값, 기준방향, 가시거리)
        }
        void RPYcallback(const std_msgs::Float32MultiArray::ConstPtr& input){
            heading_now = input->data[2]; // update sensor yaw value
        }
};
 
 
int main(int argc, char **argv){
 
   ros::init(argc, argv, "listener");
   RidarDir RD;  
   ros::spin();
   return 0;
}