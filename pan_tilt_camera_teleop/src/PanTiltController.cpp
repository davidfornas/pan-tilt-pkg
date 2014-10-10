/*
 * Pan tilt camera controller. Todas las funciones que tienen start&stop
 * se utilizan para empezar un movimiento y pararlo al tiempo.
 *
 *  Created on: 15/11/2012
 *      Author: dfornas
 */
#include <pan_tilt_camera_teleop/PanTiltController.h>
using namespace std;

PanTiltController::PanTiltController(){
openSerialPort();
configureSerialPort();

}

void PanTiltController::openSerialPort()
{
  //Se debe poder cambiar la ruta en los parámetros.
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

  if(fd == -1) // if open is unsucessful
  {
    perror("open_port: Unable to open /dev/ttyUSB0 - ");
  }
  else{fcntl(fd, F_SETFL, FNDELAY);}
}

void PanTiltController::configureSerialPort()      // configure the port
{
  struct termios port_settings; // structure to store the port settings in
  tcgetattr(fd, &port_settings);  //Get current status
  cfsetispeed(&port_settings, B9600);  // set baud rates
  cfsetospeed(&port_settings, B9600);

  //port_settings.c_cflag |= (CLOCAL|CREAD);//Not owner, reading
  port_settings.c_cflag &= ~PARENB; // set no parity,1 stop bits,8 data bits
  //port_settings.c_cflag |= CSTOPB;
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  tcsetattr(fd, TCSANOW, &port_settings); // Flush and apply the settings to the port
}

void PanTiltController::sendCommand(string command){

  int n;
  char start=0x02,stop=0x03;
  char msg[128]= " AD01;GCF:202136C:2022000 ";
  string stx = string(1,start);  
  string etx = string(1,stop);  
  string fullCommand=stx+"AD01;GC"+command+etx;
  //cout << fullCommand.data()<<endl;
  
  n = write(fd,fullCommand.data(),fullCommand.length()); 
  if(n<fullCommand.length()) cerr << "Error de escritura";
}

void PanTiltController::setPreset(string presetNumber){
  int n;
  char start=0x02,stop=0x03;
  string stx = string(1,start);  
  string etx = string(1,stop);  
  string command = "F:2021540:2022"+presetNumber+"0"; 
  string fullCommand=stx+"AD01;GC"+command+etx;
  cout << fullCommand.data()<<endl;
  n = write(fd,fullCommand.data(),fullCommand.length()); 
  if(n<fullCommand.length()) cerr << "Error de escritura";

  command = "7:2021542"; 
  fullCommand=stx+"AD01;GC"+command+etx;
  cout << fullCommand.data()<<endl;
  n = write(fd,fullCommand.data(),fullCommand.length()); 
  if(n<fullCommand.length()) cerr << "Error de escritura";

  command = "7:2021543"; 
  fullCommand=stx+"AD01;GC"+command+etx;
  cout << fullCommand.data()<<endl;
  n = write(fd,fullCommand.data(),fullCommand.length()); 
  if(n<fullCommand.length()) cerr << "Error de escritura";
}

void PanTiltController::callPreset(string presetNumber){
  int n;
  char start=0x02,stop=0x03;
  string stx = string(1,start);  
  string etx = string(1,stop);  
  string command = "F:2021400:2022"+presetNumber+"0"; 
  string fullCommand=stx+"AD01;GC"+command+etx;
  cout << fullCommand.data()<<endl;
  n = write(fd,fullCommand.data(),fullCommand.length()); 
  if(n<fullCommand.length()) cerr << "Error de escritura";
}

void PanTiltController::irisOpenStart()   //Send desired SPEED. 
{
  string command="7:0021002";
  sendCommand(command);
}

void PanTiltController::irisOpenStop()   //Send desired SPEED. 
{
  string command="7:0021004";
  sendCommand(command);
}

void PanTiltController::irisCloseStart()   //Send desired SPEED. 
{
  string command="7:0021003";
  sendCommand(command);
}

void PanTiltController::irisCloseStop()   //Send desired SPEED. 
{
  string command="7:0021004";//ESTE COMANDO ESTA REPETIDO!!
  sendCommand(command);
}

void PanTiltController::teleStart()   //Send desired SPEED. 
{
  string command="7:2021228";
  sendCommand(command);
}

void PanTiltController::teleStop()   //Send desired SPEED. 
{
  string command="7:2021224";
  sendCommand(command);
}

void PanTiltController::wideStart()   //Send desired SPEED. 
{
  string command="7:202122C";
  sendCommand(command);
}

void PanTiltController::wideStop()   //Send desired SPEED. 
{
  string command="7:2021224";
  sendCommand(command);
}

void PanTiltController::focusNearStart()   //Send desired SPEED. 
{
  string command="7:202122E";
  sendCommand(command);
}

void PanTiltController::focusNearStop()   //Send desired SPEED. 
{
  string command="7:2021224";
  sendCommand(command);
}

void PanTiltController::focusFarStart()   //Send desired SPEED. 
{
  string command="7:202122A";
  sendCommand(command);
}

void PanTiltController::focusFarStop()   //Send desired SPEED. 
{
  string command="7:2021224";
  sendCommand(command);
}

void PanTiltController::rightAndDown()   //Send desired SPEED. 
{
  string command="F:202136D:2022000";
  sendCommand(command);
}

void PanTiltController::rightAndUp()   //Send desired SPEED. 
{
  string command="F:202136B:2022000";
  sendCommand(command);
}

void PanTiltController::leftAndDown()   //Send desired SPEED. 
{
  string command="F:202136F:2022000";
  sendCommand(command);
}

void PanTiltController::leftAndUp()   //Send desired SPEED. 
{
  string command="F:2021369:2022000";
  sendCommand(command);
}

void PanTiltController::right()   //Send desired SPEED. 
{
  string command="F:202136C:2022000";
  sendCommand(command);
}

void PanTiltController::left()   //Send desired SPEED. 
{
  string command="F:2021368:2022000";
  sendCommand(command);
}

void PanTiltController::tiltup()   //Send desired SPEED. 
{
  string command="F:202136A:2022000";
  sendCommand(command);
}

void PanTiltController::tiltdown()   //Send desired SPEED. 
{
  string command="F:202136E:2022000";
  sendCommand(command);
}

void PanTiltController::stopPanTilt()   //Send desired SPEED. 
{
  string command="F:2021364:2022000";
  sendCommand(command);
}


