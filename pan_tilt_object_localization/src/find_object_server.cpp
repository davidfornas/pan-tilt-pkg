#include <time.h>

#include "ros/ros.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include "pan_tilt_object_localization/FindAnObject.h"
#include <pan_tilt_camera_teleop/PanTiltController.h>

#define SECONDS 43

PanTiltController _ptc;
time_t _initTime;

void turnCam()
{
  time_t current;
  
  while ((time(&current) - _initTime) < SECONDS) 
  {
    if((time(&current) % 3) == 0)
    {
      _ptc.left();
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
      _ptc.stopPanTilt();
    }
  }
    /*if(time(&current) % 2 == 0) _ptc.left();
    else _ptc.stopPanTilt();
  }
  _ptc.stopPanTilt();
  system("rosnode kill /surfv");*/
}

bool searchObject(pan_tilt_object_localization::FindAnObject::Request  &req,
                  pan_tilt_object_localization::FindAnObject::Response &res)
{
  time(&_initTime);
	ROS_INFO("request: tiempo de busqueda %ld", (long int)req.time);

  boost::thread turn_cam(&turnCam);
  turn_cam.join();
	
	ROS_INFO("sending back response: final de la busqueda");
  return true;
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_object");
  ros::NodeHandle n;
 
  ros::ServiceServer service = n.advertiseService("find_time", searchObject);
  ROS_INFO("Ready to search objects.");
  ros::spin();
 
  return 0;
}
