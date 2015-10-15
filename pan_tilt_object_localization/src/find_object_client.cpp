#include "ros/ros.h"
#include "pan_tilt_object_localization/FindAnObject.h"
#include <cstdlib>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "find_object_client");
   if (argc != 2)
   {
     ROS_INFO("usage: find_object_server T");
     return 1;
   }
   
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<pan_tilt_object_localization::FindAnObject>("find_time");
   pan_tilt_object_localization::FindAnObject srv;
   srv.request.time = atoll(argv[1]);

   if (client.call(srv)) ROS_INFO("Object LOCATED");
   else
   {
     ROS_ERROR("Object NOT LOCATED");
     return 1;
   }
  
   return 0;
}