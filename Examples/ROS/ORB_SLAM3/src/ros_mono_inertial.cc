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
std::mutex m_buf;
cv::Mat M1l,M2l,M1r,M2r;
double pretime=0;
ORB_SLAM3::System* mpSLAM;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  m_buf.lock();
  img0_buf.push(img_msg);
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

    cv::Mat im;
    std_msgs::Header header;
    double time = 0;
    //make sure got enough imu frame before a image frame
    if (!img0_buf.empty()&&!imu_buf.empty())
    {
      // m_buf.lock();
      time = img0_buf.front()->header.stamp.toSec();
      header = img0_buf.front()->header;
      im = getImageFromMsg(img0_buf.front());
      img0_buf.pop();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      if(!imu_buf.empty())
      {
        // Load imu measurements from previous frame
        vImuMeas.clear();
        while(imu_buf.empty() == 0 && imu_buf.front()->header.stamp.toSec()<=time&&imu_buf.front()->header.stamp.toSec()>pretime)
        {
          double t = imu_buf.front()->header.stamp.toSec();
          double dx = imu_buf.front()->linear_acceleration.x;
          double dy = imu_buf.front()->linear_acceleration.y;
          double dz = imu_buf.front()->linear_acceleration.z;
          double rx = imu_buf.front()->angular_velocity.x;
          double ry = imu_buf.front()->angular_velocity.y;
          double rz = imu_buf.front()->angular_velocity.z;
          // printf("%f %f %f %f %f %f %f \n",dx,dy,dz,rx,ry,rz,t);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(dx,dy,dz,rx,ry,rz,t));
          imu_buf.pop();
        }
      }
      // printf("%f \n\n",time);
      pretime = time;

      mpSLAM->TrackMonocular(im,time,vImuMeas);

      // m_buf.unlock();
    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  if(argc != 3)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings" << endl;
    ros::shutdown();
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  std::cout << std::string(argv[1]) << "\n" << std::string(argv[2]) << std::endl;
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);
  mpSLAM = &SLAM;


  ROS_WARN("waiting for image and imu...");



//  ros::Subscriber sub_imu = n.subscribe("/kitti/oxts/imu", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
//  ros::Subscriber sub_img0 = n.subscribe("/kitti/camera_color_left/image_raw", 100, img0_callback);

//  ros::Subscriber sub_imu = n.subscribe("/airsim/imu", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
//  ros::Subscriber sub_img0 = n.subscribe("/airsim/rgb_img", 100, img0_callback);

  ros::Subscriber sub_imu = n.subscribe("/camera/imu", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_img0 = n.subscribe("/camera/color/image_raw", 100, img0_callback);

  std::thread sync_thread{sync_process};

  ros::spin();

  return 0;
}