#include "ros/ros.h"
#include "pan_tilt_object_localization/FindAnObject.h"

#include <pan_tilt_camera_teleop/PanTiltController.h>

PanTiltController _ptc;

bool searchObject(pan_tilt_object_localization::FindAnObject::Request  &req,
                  pan_tilt_object_localization::FindAnObject::Response &res)
{

	ROS_INFO("request: tiempo de busqueda %ld", (long int)req.time);

  //aquí irá el movimiento de la camara y la ejecucion del paquete object_localization

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