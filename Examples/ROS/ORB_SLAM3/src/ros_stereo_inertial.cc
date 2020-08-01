/**
* This file is added by xiefei2929@126.com
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h> 

#include"../../../include/System.h"

using namespace std;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;
cv::Mat M1l,M2l,M1r,M2r;

ORB_SLAM3::System* mpSLAM;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }


    cv::Mat img = cv_ptr->image.clone();
    return img;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    return;
}

void sync_process()
{
    while(1)
    {
        
        cv::Mat imLeft, imRight, imLeftRect, imRightRect;
        std_msgs::Header header;
        double time = 0;
        //make sure got enough imu frame before a image frame
        if (!img0_buf.empty() && !img1_buf.empty()&&imu_buf.size()>15)
        {
            double time0 = img0_buf.front()->header.stamp.toSec();
            double time1 = img1_buf.front()->header.stamp.toSec();
            // 0.003s sync tolerance
            if(time0 < time1 - 0.003)
            {
                img0_buf.pop();
                printf("throw img0\n");
            }
            else if(time0 > time1 + 0.003)
            {
                img1_buf.pop();
                printf("throw img1\n");
            }
            else
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                imLeft = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
                imRight = getImageFromMsg(img1_buf.front());
                img1_buf.pop();
                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                if(!imu_buf.empty())
                {
                    // Load imu measurements from previous frame
                    vImuMeas.clear();
          
                    while(imu_buf.front()->header.stamp.toSec()<=time&&imu_buf.front()->header.stamp.toSec()>time-1)
                    {
               
                        double t = imu_buf.front()->header.stamp.toSec();
                        double dx = imu_buf.front()->linear_acceleration.x;
                        double dy = imu_buf.front()->linear_acceleration.y;
                        double dz = imu_buf.front()->linear_acceleration.z;
                        double rx = imu_buf.front()->angular_velocity.x;
                        double ry = imu_buf.front()->angular_velocity.y;
                        double rz = imu_buf.front()->angular_velocity.z;
                        
                        vImuMeas.push_back(ORB_SLAM3::IMU::Point(dx,dy,dz,rx,ry,rz,t));
                        imu_buf.pop();
                    }
                }
                cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
                cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);
                mpSLAM->TrackStereo(imLeftRect,imRightRect,time,vImuMeas);

            }

        }
        //m_buf.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stero_inertial");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);
    mpSLAM = &SLAM;
        // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }


    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


    ROS_WARN("waiting for image and imu...");



    ros::Subscriber sub_imu = n.subscribe("/imu0", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_img0 = n.subscribe("/cam0/image_raw", 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe("/cam1/image_raw", 100, img1_callback);


    std::thread sync_thread{sync_process};
   
    ros::spin();

    return 0;
}


