#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class bird_view{
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter left_sub;
    image_transport::SubscriberFilter right_sub;
    cv::Mat left_image;
    cv::Mat right_image;
    cv::Mat rear_image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer< MySyncPolicy > sync;

  
  public:
    bird_view():
        it(nh),
        left_sub(it,"/left_pi/camera/image", 1),
        right_sub(it,"/right_pi/camera/image", 1),
        sync( MySyncPolicy( 50 ), left_sub, right_sub )
        {
            sync.registerCallback( boost::bind( &bird_view::imageCallback, this, _1, _2 ) );
        }   
    
    void imageCallback(const sensor_msgs::ImageConstPtr& left_msg,const sensor_msgs::ImageConstPtr& right_msg)
    {
      try
      {
        left_image = cv_bridge::toCvShare(left_msg, "bgr8")->image;
     // }
     // catch (cv_bridge::Exception& e)
     // {
     //   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", left_msg->encoding.c_str());
     // }
     // try
     // {
        right_image = cv_bridge::toCvShare(right_msg, "bgr8")->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", right_msg->encoding.c_str());
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
  
  bird_view a;
  cv::namedWindow("left_view");
  cv::namedWindow("right_view");
  cv::startWindowThread();
  ros::Rate fps(15);
  while(ros::ok()){
    ros::spinOnce();
    a.show_image();
    fps.sleep();
  }
  

  cv::destroyWindow("view");
}
