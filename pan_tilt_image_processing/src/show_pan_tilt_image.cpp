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
//bool _dibuja;
std::vector<cv::Point2f> _scene_corners(4);

void drawLines()
{
  line(cv_ptr->image,_scene_corners[0], _scene_corners[1], cv::Scalar(0, 255,0),4);
  line(cv_ptr->image,_scene_corners[1], _scene_corners[2], cv::Scalar(0, 255,0),4);
  line(cv_ptr->image,_scene_corners[2], _scene_corners[3], cv::Scalar(0, 255,0),4);
  line(cv_ptr->image,_scene_corners[3], _scene_corners[0], cv::Scalar(0, 255,0),4);
}

void showIPTCB(const sensor_msgs::ImageConstPtr& msg)
  {
    try { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //if (_dibuja)
    //{  
      drawLines();
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(1);
    //}
  }

void paintBox(const std_msgs::Float32MultiArray& points)
{
  //_dibuja = false;

  for (int i = 0; i < 4; i++)
  {
    _scene_corners[i].x = points.data[i*2];
    _scene_corners[i].y = points.data[i*2+1];
  }
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(1);
}

/*void killedSurf(const std_msgs::String& kill)
{
  if(kill.data.compare("Off") == 0) _dibuja = true;
}*/

void initCorners() 
{ 
  for (int i = 0; i < 4; i++)
  {
    _scene_corners[i].x = 0;
    _scene_corners[i].y = 0;
  }
}

int main(int argc, char** argv)
{
  initCorners();
  ros::init(argc, argv, "show_pan_tilt_image");
  ros::NodeHandle n;
  ros::Subscriber impt = n.subscribe("/panTilt/image_raw", 1, showIPTCB);
  ros::Subscriber points = n.subscribe("/surf/corners_object", 1, paintBox);
  //ros::Subscriber killed = n.subscribe("/stop/surf", 1, killedSurf);
  ros::spin();
  /*while(ros::ok()){
    ros::spinOnce(); 
  }*/

  return 0;
}