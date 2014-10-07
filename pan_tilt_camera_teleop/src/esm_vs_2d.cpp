#include "PanTiltController.h"
#include "time.h"

#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <mar_perception/ESMTracking.h>
#include <mar_perception/VirtualImage.h>

#include <geometry_msgs/Twist.h>

#define SAT	35

int main(int argc, char **argv){

  if (argc!=7) {
  	std::cerr << "Usage: " << argv[0] << " <image_topic> <image_info_topic> <output_topic> <gain> <minimum> <speed>" << std::endl;
	exit(0);
  }

  std::string image_topic(argv[1]);
  std::string image_info_topic(argv[2]);
  std::string output_topic(argv[3]);
  double GAIN=atof(argv[4]);
  double MINIMUM=atof(argv[5]);
  double SPEED=atof(argv[6]);

  ros::init(argc, argv, "esm_vs_2d");
  ros::NodeHandle nh;
 
  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(output_topic, 1);

  vpImage<vpRGBa> Ic; // Color image

  // Declare a framegrabber able to read ROS images
  VirtualImage g(nh,image_topic,image_info_topic);
  while (!g.ready() && ros::ok()) {ros::spinOnce();}

  // Open the framegrabber by loading the first image of the sequence
  g.open(Ic) ;
  g.acquire(Ic);
  vpDisplayX window(Ic);
  vpDisplay::display(Ic);
  vpDisplay::flush(Ic);
  
  PanTiltController ptc;

  ESMTracking esm(&Ic);
  ros::Rate loop_rate(10);
  while(ros::ok())
  {

    std::cerr << "ros spin" << std::endl;
    ros::spinOnce();

    g.acquire(Ic);
    esm.perceive();

    vpDisplay::display(Ic);
    esm.draw(Ic);

    //Compute control law and send output
    geometry_msgs::Twist msg;
    vpImagePoint ip=esm.getCentroid();

    double angle=atan2(esm.pEnd[4]-esm.pEnd[2],esm.pEnd[5]-esm.pEnd[3]);

    double errorx=GAIN*(Ic.getRows()/2.0-ip.get_v());
    double errory=-GAIN*(Ic.getCols()/2.0-ip.get_u());
    if (errorx>SAT) errorx=SAT; else if (errorx<-SAT) errorx=-SAT;
    if (errory>SAT) errory=SAT; else if (errory<-SAT) errory=-SAT;

    //Perform control law
    if(errorx>MINIMUM){ ptc.tiltup();
    usleep(SPEED);
    ptc.stopPanTilt();}
    else if(errorx<-MINIMUM){ptc.tiltdown();
    usleep(SPEED);
    ptc.stopPanTilt();}

    if(errory>MINIMUM){ ptc.left();
    usleep(SPEED);
    ptc.stopPanTilt();}
    else if(errory<-MINIMUM){ptc.right();
    usleep(SPEED);
    ptc.stopPanTilt();}

    msg.linear.x=errorx;
    msg.linear.y=-errory;
    msg.linear.z=0;
    msg.angular.x=0;
    msg.angular.y=0;
    msg.angular.z=-55*angle;
    pub.publish(msg);

    vpDisplay::flush(Ic);
    loop_rate.sleep();
  }

}

