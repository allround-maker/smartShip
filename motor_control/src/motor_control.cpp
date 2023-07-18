#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <limits.h>
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>


class Motor_control
{
    private:
        ros::NodeHandle n_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
    private:

        vector<float> Go_straight(vector<float> obstacle_avoiding)
        {   
            vector<float> control;
            if()
        }
        vector<float> Situation(vector<float> obstacle_avoiding)
        {   
            vector<float> situation; //[left,right,angle,time]
            if(obstacle_avoiding[0] <= 1 && obstacle_avoiding[0] >= -1)
            {
                
            }
            return situation;
        }
        
}