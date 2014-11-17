#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <cstdlib>


std::mutex bloqueo;

void permisos_ttyUSB0(){
	//bloqueo.lock();
	system("sudo chmod 777 /dev/ttyUSB0");
}

void rViz(){
	//system("rosrun rviz rviz");
	system("rosrun image_view image_view image:=panTilt/image_raw");
}

void grabber(){
	system("roslaunch pan_tilt_camera_grabber PanTiltGrabberRaw.launch");
}

void teleop(){
	system("rosrun pan_tilt_camera_teleop keyboard");
}

int main(int argc, char **argv){
	std::thread t1(permisos_ttyUSB0);
	t1.join();
	
	std::thread t3(grabber);
	std::thread t2(rViz);
	std::thread t4(teleop);
	
	//system("roslaunch pan_tilt_camera_grabber PanTiltGrabberRaw.launch");
	t3.join();
	t2.join();
	sleep(2000);
	//system("rosrun pan_tilt_camera_teleop keyboard");
	t4.join();
		
	return 0;
}

