/////////////////////////////////////////////////
/*
 * Pan tilt camera controller. Todas las funciones que tienen start&stop
 * se utilizan para empezar un movimiento y pararlo al tiempo.
 *
 *  Created on: 15/11/2012
 *      Author: dfornas
 */

#include "iostream"
#include "string"

#include <stdio.h> // standard input / output functions
#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

using namespace std;

class PanTiltController {
  public:
    ///Abre el puerto serie y lo configura
    PanTiltController();


    ///Funciones de apertura y cierre del iris.
    void irisOpenStart();
    void irisOpenStop();
    void irisCloseStart();
    void irisCloseStop();
    
    ///Alejar el zoom
    void teleStart();
    void teleStop();
    
    ///Acecar el zoom
    void wideStart();
    void wideStop();
    
    ///Acercar el enfoque
    void focusNearStart();
    void focusNearStop();
    
    ///Alejar el enfoque
    void focusFarStart();
    void focusFarStop();
    
    ///Iniciar un movimiento en la dirección deseada
    void rightAndDown();
    void rightAndUp();
    void leftAndDown();
    void leftAndUp();
    void right();
    void left();
    void tiltup();
    void tiltdown();
    ///Detener el movimiento iniciado
    void stopPanTilt();

    void setPreset(string presetNumber);//it can store 1-64 positions.
    void callPreset(string presetNumber);//go to one of those preset positions.

  private:
      int fd;
      void openSerialPort();
      void configureSerialPort();
      void sendCommand(string command);


};

