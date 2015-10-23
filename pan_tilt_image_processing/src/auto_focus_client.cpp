#include "ros/ros.h"
#include "pan_tilt_image_processing/Focusing.h"
#include <cstdlib>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "auto_focus_client");
   if (argc != 2)
   {
     ROS_INFO("usage: auto_focus_server T");
     return 1;
   }
   
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<pan_tilt_image_processing::Focusing>("auto_focus");
   pan_tilt_image_processing::Focusing srv;
   srv.request.time = atoll(argv[1]);

   if (client.call(srv)) ROS_INFO("Image FOCUSED");
   else
   {
     ROS_ERROR("Image NOT FOCUSED");
     return 1;
   }
  
   return 0;
}