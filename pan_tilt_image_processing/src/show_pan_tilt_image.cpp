#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

static const std::string OPENCV_WINDOW = "Image Pan-Tilt";
cv_bridge::CvImagePtr cv_ptr; 

void showIPTCB(const sensor_msgs::ImageConstPtr& msg)
  {
    try { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
      
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "show_pan_tilt_image");
  ros::NodeHandle n;
  ros::Subscriber impt = n.subscribe("/panTilt/image_raw", 1, showIPTCB);
  ros::spin();  

  return 0;
}