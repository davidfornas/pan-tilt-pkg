#include "PanTiltController.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


ros::Subscriber err_sub;
PanTiltController ptc;


void onError(const geometry_msgs::Twist & msg)
{
  if(msg.linear.x<0)
    ptc.tiltup();
  else if(msg.linear.x>0)
    ptc.tiltdown();
  else
    return;
  //                stopCamera();
  //        
  //                ptc.tiltup();
  //                ptc.tiltdown();
  //                ptc.left();
  //                ptc.right();
}


int main(int argc, char** argv)
{
  ptc.setPreset("01");
  ros::init(argc,argv,"pt");
  ros::NodeHandle n;
  err_sub=n.subscribe("error",1,onError); 

  ros::spin();
  ptc.stopPanTilt();


  return(0);
}


