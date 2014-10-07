/////////////////////////////////////////////////
// Pan Tilt camera controller                  //
/////////////////////////////////////////////////

#include "iostream"
#include "string"

#include <stdio.h> // standard input / output functions
#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

using namespace std;

class PanTiltController {
  public:
    PanTiltController();

    void irisOpenStart();
    void irisOpenStop();
    void irisCloseStart();
    void irisCloseStop() ;
    void teleStart()  ;
    void teleStop()  ;
    void wideStart();
    void wideStop();
    void focusNearStart();
    void focusNearStop();
    void focusFarStart();
    void focusFarStop();
    void rightAndDown();
    void rightAndUp();
    void leftAndDown();
    void leftAndUp();
    void right();
    void left();
    void tiltup();
    void tiltdown();
    void stopPanTilt();

    void setPreset(string presetNumber);//it can store 1-64 positions.
    void callPreset(string presetNumber);//go to one of those preset positions.

  private:
      int fd;
      void openSerialPort();
      void configureSerialPort();
      void sendCommand(string command);


};

