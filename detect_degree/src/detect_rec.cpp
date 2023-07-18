#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include "std_msgs/Int32.h"
using namespace std;
using namespace cv;

int a = 0;
int b = 0;
int c = 0;
int j = 0;

float zone_1a = -90;
float zone_1b = -30;

float zone_2a = -30;
float zone_2b = 30;

float zone_3a = 30;
float zone_3b = 90;

int dock = 0;

int main (int argc, char** argv){
	ros::init(argc, argv, "detect_degree");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("degree_rec", 10);
    ros::Rate loop_rate(10);
    std_msgs::Int32 msg;
	
	cv::VideoCapture cap(4);	
	cv::Mat frame;

    int ddepth = CV_16S;
	
	if(cap.isOpened()){
		while(j == 0){
			cap >> frame;

            int x1 = 0;
            int x2 = 1280;
            int y1 = 100;
            int y2 = 600;

            //frame = frame(Range(y1,y2), Range(x1, x2));

            cv::cvtColor(frame,frame, cv::COLOR_RGB2BGR);
            cv::cvtColor(frame,frame, cv::COLOR_BGR2HSV);

            cv::Scalar lower_blue = cv::Scalar(90,50,70);
            cv::Scalar upper_blue = cv::Scalar(128,255,255);

            cv::inRange(frame, lower_blue, upper_blue, frame);

            //cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            //cout << frame.channels() << endl;
			//cv::imshow("streaming_video", frame);

            cv::Mat scharr_x, scharr_y;

            cv::Scharr(frame, scharr_x, ddepth, 1,0);
            cv::Scharr(frame, scharr_y, ddepth, 0,1);

            cv::Mat Canny_frame;
            
            cv::Canny(scharr_x, scharr_y, frame, 500, 1000);

            //cv::imshow("Canny", Canny_frame);

            vector<vector<Point> > contours;
            findContours(frame, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            //cout << contours.size()<<endl;

            cv::Mat frame_result;
            vector<Point2f> approx;

            frame_result = frame.clone();

            vector<Moments> mu(contours.size());
            for (int i =0; i < contours.size(); i++)
            {
                mu[i] = moments(contours[i], false);
            }

            vector<Point2f> mc(contours.size());
            for (int i =0; i < contours.size(); i++)
            {
                mc[i] = Point2f(mu[i].m10/mu[i].m00,mu[i].m01/mu[i].m00);
            }
            
            
            for (size_t i =0; i < contours.size(); i++)
            {   //cout<<"ContourArea : " <<contours.size()<<endl;
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
                float degree = ((640 - mu[i].m10/mu[i].m00) * 0.078);

                



                if (fabs(contourArea(Mat(approx))) > 1000)
                {   //cout <<"ContourArea : " << fabs(contourArea(Mat(contours[i]))) << endl;
                    int size = approx.size();
                    if (size%2 ==0)
                    {
                        line(frame_result, approx[0], approx[approx.size()-1], Scalar(0,255,0),3);

                        for (int k=0; k<size-1; k++)
                            line(frame_result, approx[k], approx[k+1], Scalar(0,255,0),3);

                        for (int k=0; k<size; k++)
                            circle(frame_result, approx[k], 3, Scalar(0,0,255));

                    }
                    else {
                        line(frame_result, approx[0], approx[approx.size()-1], Scalar(0,255,0),3);

                        for (int k=0; k<size-1; k++)
                            line(frame_result, approx[k], approx[k+1], Scalar(0,255,0), 3);

                        for (int k=0; k<size; k++)
                            circle(frame_result, approx[k],3, Scalar(0,0,255));
                    }

                    if (size == 3);                    

                    else if (size == 4)
                    {
                    circle(frame_result, mc[i], 4, Scalar(0,255,0), -1,8,0);
                    //cout << "moments : " << mc[i] << endl;
                    //cout << "degree : "<< degree << endl;
                    if (zone_1a <= degree <= zone_1b)
                            a += 1;

                        else if ( zone_2a <= degree <= zone_2b )
                            b += 1;

                        else if ( zone_3a <= degree <= zone_3b )
                            c += 1;
                        

                        if (a == 10)
                            dock =1;
                        
                        else if (b == 10)
                            dock =2;

                        else if (c == 10)
                            dock =3;


                        if (dock >0){
                            msg.data = dock;
                            
                            ROS_INFO("%f", msg.data);

                            chatter_pub.publish(msg);

                            j++;
                            ros::spinOnce();
                            loop_rate.sleep();}

                    else ;
                    //ROS_INFO("%f", msg.data);
                }
                }
            }
		}
	}
	
	else{
		std::cout << "NO FRAME, CHECK YOUR CAMERA!" << std::endl;
	}	
	
	//cv::destroyAllWindows();
	
	return 0;
}