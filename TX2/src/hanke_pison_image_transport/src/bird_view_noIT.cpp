#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>

class bird_view{
  private:
    cv::Mat left_image;
    cv::Mat right_image;
    cv::Mat rear_image;
    ros::Subscriber left_sub;
    ros::Subscriber right_sub;

  public:
    bird_view(ros::NodeHandle& left_it, ros::NodeHandle& right_it){
      left_sub = left_it.subscribe("camera/image", 1, &bird_view::left_imageCallback, this);
      right_sub = right_it.subscribe("camera/image", 1, &bird_view::right_imageCallback, this);

    }

    void left_imageCallback(const sensor_msgs::ImagePtr& msg)
    {
      try
      {
        left_image = cv_bridge::toCvShare(msg, "bgr8")->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }

    void right_imageCallback(const sensor_msgs::ImagePtr& msg)
         {
      try
      {
        right_image = cv_bridge::toCvShare(msg, "bgr8")->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }

    void rear_imageCallback(const sensor_msgs::ImageConstPtr& msg)
         {
      try
      {
        rear_image = cv_bridge::toCvShare(msg, "bgr8")->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }


    void show_image(){
      if(!left_image.empty())
  	cv::imshow("left_view", left_image);
      if(!right_image.empty())
        cv::imshow("right_view", right_image);

      cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle left_pi("/left_pi");
  ros::NodeHandle right_pi("/right_pi");
  
//  bird_view a(nh);
  cv::namedWindow("left_view");
  cv::namedWindow("right_view");
  cv::startWindowThread();
  bird_view a(left_pi, right_pi);
//  image_transport::ImageTransport left(nh);
//  image_transport::Subscriber left_sub = right.subscribe("/left_pi/camera/image", 1, &bird_view::left_imageCallback, &a);
//  image_transport::Subscriber right_sub = left.subscribe("/right_pi/camera/image", 1, &bird_view::right_imageCallback, &a);
  ros::AsyncSpinner spinner(6);
  spinner.start();
  ros::Rate fps(15);
  while(ros::ok()){
    //ros::spinOnce();
    a.show_image();
    fps.sleep();
  }
  

  cv::destroyWindow("view");
}
