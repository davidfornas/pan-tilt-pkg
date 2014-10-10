/*
 * Este programa es un ejemplo de servidor para ROS que realiza movimientos
 * de pan y tilt a petición de un mensaje de ROS cada 5hz.
 *
 *  Created on: 15/11/2012
 *      Author: dfornas
 */
#include <stdio.h>
#include <stdlib.h>

#include <pan_tilt_camera_teleop/PanTiltController.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Subscriber pt_sub;
PanTiltController ptc;
sensor_msgs::JointState js;

///Empezar el movimiento indicado por el mensaje
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

///Si el mensaje es correcto lo copia
void moveCamera(const sensor_msgs::JointState & mv)
{
  ROS_ERROR("RECIBIDO");
  if(mv.position.size()!=2)return;
  js=mv;
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

