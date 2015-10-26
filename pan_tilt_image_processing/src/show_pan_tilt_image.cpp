#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/String.h>
#include <fstream>
#include <boost/thread.hpp>

static const std::string OPENCV_WINDOW = "Image Pan-Tilt";
cv_bridge::CvImagePtr cv_ptr; 
bool _dibuja;

void showIPTCB(const sensor_msgs::ImageConstPtr& msg)
  {
    try { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //findNode();
    if (_dibuja)
    {  
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(1);
    }
  }

void paintBox(const std_msgs::Float32MultiArray& points)
{
  _dibuja = false;
  std::vector<cv::Point2f> scene_corners(4);

  /*for (int i = 0; i < 8; i++)
  {
    //if (i % 2 != 0) scene_corners[i/2].x = points.data[i]
    scene_corners[i].x = points.data[i++];
    scene_corners[i].y = points.data[i];
  }*/
  
    scene_corners[0].x = points.data[0];
    scene_corners[0].y = points.data[1];
    scene_corners[1].x = points.data[2];
    scene_corners[1].y = points.data[3];
    scene_corners[2].x = points.data[4];
    scene_corners[2].y = points.data[5];
    scene_corners[3].x = points.data[6];
    scene_corners[3].y = points.data[7];

  line(cv_ptr->image,scene_corners[0], scene_corners[1], cv::Scalar(0, 255,0),4);
  line(cv_ptr->image,scene_corners[1], scene_corners[2], cv::Scalar(0, 255,0),4);
  line(cv_ptr->image,scene_corners[2], scene_corners[3], cv::Scalar(0, 255,0),4);
  line(cv_ptr->image,scene_corners[3], scene_corners[0], cv::Scalar(0, 255,0),4);
  
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(1);
}

void killedSurf(const std_msgs::String& kill)
{
  if(kill.data.compare("Off") == 0) _dibuja = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "show_pan_tilt_image");
  ros::NodeHandle n;
  ros::Subscriber impt = n.subscribe("/panTilt/image_raw", 1, showIPTCB);
  ros::Subscriber points = n.subscribe("/surf/corners_object", 1, paintBox);
  ros::Subscriber killed = n.subscribe("/stop/surf", 1, killedSurf);
  ros::spin();
  /*while(ros::ok()){
    ros::spinOnce(); 
  }*/

  return 0;
}