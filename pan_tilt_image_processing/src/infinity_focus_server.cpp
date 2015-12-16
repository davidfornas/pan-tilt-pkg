#include <ros/ros.h>
#include <boost/thread.hpp>

#include <pan_tilt_camera_teleop/PanTiltController.h>
#include "pan_tilt_image_processing/InfinityFocus.h"

#define TIMEOUT 7
 
static const std::string OPENCV_WINDOW = "Image window";
bool _timeout = true;
PanTiltController _ptc;


/**Enfoque al infinito, 7 segundos es lo que tarda desde el enfoque proximo*/
void infinityFocus(){
	_ptc.focusFarStart();
	while (_timeout){}
	_ptc.focusFarStop();
}

void closeFocus(){
	_ptc.focusNearStart();
	while (_timeout){}
	_ptc.focusNearStop();
}

bool focusNow(pan_tilt_image_processing::InfinityFocus::Request  &req,
		 	  pan_tilt_image_processing::InfinityFocus::Response &res)
{
	boost::thread enfocando;
	ROS_INFO("Enfocando");
	if(req.order == 1) enfocando = boost::thread(&infinityFocus);
	if(req.order == 2) enfocando = boost::thread(&closeFocus);
	
	sleep(TIMEOUT);
	_timeout = false;
	
	enfocando.join();

	ROS_INFO("ENFOCADO!!");
	_timeout = true;

	return 1;
}
  
int main (int argc, char** argv){
	
	ros::init(argc, argv, "infinity_focus");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("infinity_focus", focusNow);
	
	ros::spin();

	return 0;
}
