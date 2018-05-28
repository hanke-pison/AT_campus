#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class bird_view{
  private:
    cv::Mat left_image;
    cv::Mat right_image;
    cv::Mat rear_image;
    image_transport::Subscriber left_sub;
    image_transport::Subscriber right_sub;

  public:
    bird_view(image_transport::ImageTransport& it){
      left_sub = it.subscribe("/left_pi/camera/image", 1, &bird_view::left_imageCallback, this);
      right_sub = it.subscribe("/right_pi/camera/image", 1, &bird_view::right_imageCallback, this);

    }

    void left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    void right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
  ros::NodeHandle nh;
  
//  bird_view a(nh);
  cv::namedWindow("left_view");
  cv::namedWindow("right_view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  bird_view a(it);
//  image_transport::ImageTransport left(nh);
//  image_transport::Subscriber left_sub = right.subscribe("/left_pi/camera/image", 1, &bird_view::left_imageCallback, &a);
//  image_transport::Subscriber right_sub = left.subscribe("/right_pi/camera/image", 1, &bird_view::right_imageCallback, &a);
  
  ros::Rate fps(15);
  while(ros::ok()){
    ros::spinOnce();
    a.show_image();
    fps.sleep();
  }
  

  cv::destroyWindow("view");
}
