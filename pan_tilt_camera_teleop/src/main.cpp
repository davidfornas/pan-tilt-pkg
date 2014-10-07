#include <stdio.h>
#include <stdlib.h>

#include "PanTiltController.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Subscriber pt_sub;
PanTiltController ptc;
sensor_msgs::JointState js;

void issue_mv(){

  if(js.position.size()==0)return;
  float pan=js.position[0];
  float tilt=js.position[1];
  ROS_ERROR("PAN: %f TILT: %f", pan, tilt);
  if(tilt<0) ptc.tiltup();
  else if(tilt>0) ptc.tiltdown();
  if(pan<0) ptc.left();
  else if(pan>0) ptc.right();

}
void moveCamera(const sensor_msgs::JointState & mv)
{
  ROS_ERROR("RECIBIDO");
  if(mv.position.size()!=2)return;
  js=mv;


  //      ptc.teleStart();
  //      dirty = true;
  //      break;
  //    case KEYCODE_X:
  //      ptc.wideStart();
  //      dirty = true;
  //      break;
  //    case KEYCODE_Q:
  //      ptc.focusNearStart();
  //      dirty = true;
  //      break;
  //    case KEYCODE_E:
  //      ptc.focusFarStart();
  //      dirty = true;
  //      break;
  //    case KEYCODE_1:
  //      ptc.setPreset("01");
  //      dirty = true;
  //      break;
  //    case KEYCODE_2:
  //      ptc.setPreset("02");
  //      dirty = true;
  //      break;
  //    case KEYCODE_3:
  //      ptc.setPreset("03");
  //      dirty = true;
  //      break;
  //    case KEYCODE_4:
  //      ptc.setPreset("04");
  //      dirty = true;
  //      break;
  //    case KEYCODE_5:
  //      ptc.callPreset("01");
  //      dirty = true;
  //      break;
  //    case KEYCODE_6:
  //      ptc.callPreset("02");
  //      dirty = true;
  //      break;
  //    case KEYCODE_7:
  //      ptc.callPreset("03");
  //      dirty = true;
  //      break;
  //    case KEYCODE_8:
  //      ptc.callPreset("04");
  //      dirty = true;
  //      break;
  //    default:
  //      dirty = false;
  //  }
  //
  //}



  }

int main(int argc, char** argv)
{
  ros::init(argc,argv,"controller");
  ros::NodeHandle n;

  ptc.setPreset("01");
  pt_sub = n.subscribe("panTilt",1,moveCamera);
  ros::Rate loop_rate(5);

  while(ros::ok()){
    //Se debe modificar este esquema porque no produce movimientos precisos, sería más adecuado 
    //cuando ser recibe algo en esa función hacer ya el start & stop en función de los grados 
    //que se desea mover porque aquí según cuando se recibe el mensaje hace que 
    //el tiempo se variable porque no se frena 30 ms más tarde, sino cada 30ms se hace un stop...


    issue_mv();
    ros::spinOnce();
    loop_rate.sleep();
    ptc.stopPanTilt();
    ptc.teleStop();

  }

  ptc.stopPanTilt();
  ptc.teleStop();

  return(0);
}

