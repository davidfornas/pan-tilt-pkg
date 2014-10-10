/*
 * Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
/*
 * Este programa permite probar las posibilidades de la camara por teclado.
 * Es un ejemplo de programa que es cliente y servidor a la vez.
 *
 *  Modified on: 15/11/2012
 *       dfornas
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <pan_tilt_camera_teleop/PanTiltController.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_I 0x69
#define KEYCODE_O 0x6F
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38

//#define KEYCODE_A_CAP 0x41

class PanTiltKeyboardTeleopNode
{
  private:
    ros::NodeHandle n_;

  public:
    PanTiltKeyboardTeleopNode()
    {
      ros::NodeHandle n_private("~");
      ptc.setPreset("01");
    }

    ~PanTiltKeyboardTeleopNode() { }
    void keyboardLoop();
    PanTiltController ptc;
    void stopCamera(){
      ptc.stopPanTilt();
      ptc.teleStop();
    }

};

PanTiltKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  PanTiltKeyboardTeleopNode tbk;

  boost::thread t = boost::thread(boost::bind(&PanTiltKeyboardTeleopNode::keyboardLoop, &tbk));

  ros::spin();

  t.interrupt();
  t.join();
  tbk.stopCamera();
  tcsetattr(kfd, TCSANOW, &cooked);

  return(0);
}

void PanTiltKeyboardTeleopNode::keyboardLoop()
{
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("Use WASD keys to control the camera");

  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;

  for(;;)
  {
    boost::this_thread::interruption_point();

    // get the next event from the keyboard
    int num;

    if ((num = poll(&ufd, 1, 250)) < 0)
    {
      perror("poll():");
      return;
    }
    else if(num > 0)
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        return;
      }
    }
    else
    {
      if (dirty == true)
      {
        stopCamera();
        dirty = false;
      }

      continue;
    }

    switch(c)
    {
      case KEYCODE_W:
        ptc.tiltup();
        dirty = true;
        break;
      case KEYCODE_S:
        ptc.tiltdown();
        dirty = true;
        break;
      case KEYCODE_A:
        ptc.left();
        dirty = true;
        break;
      case KEYCODE_D:
        ptc.right();
        dirty = true;
        break;
      case KEYCODE_Z:
        ptc.teleStart();
        dirty = true;
        break;
      case KEYCODE_X:
        ptc.wideStart();
        dirty = true;
        break;
      case KEYCODE_Q:
        ptc.focusNearStart();
        dirty = true;
        break;
      case KEYCODE_E:
        ptc.focusFarStart();
        dirty = true;
        break;
      case KEYCODE_I:
        ptc.irisOpenStart();
        dirty = true;
        break;
      case KEYCODE_O:
        ptc.irisCloseStart();
        dirty = true;
        break;
      case KEYCODE_1:
        ptc.setPreset("01");
        dirty = true;
        break;
      case KEYCODE_2:
        ptc.setPreset("02");
        dirty = true;
        break;
      case KEYCODE_3:
        ptc.setPreset("03");
        dirty = true;
        break;
      case KEYCODE_4:
        ptc.setPreset("04");
        dirty = true;
        break;
      case KEYCODE_5:
        ptc.callPreset("01");
        dirty = true;
        break;
      case KEYCODE_6:
        ptc.callPreset("02");
        dirty = true;
        break;
      case KEYCODE_7:
        ptc.callPreset("03");
        dirty = true;
        break;
      case KEYCODE_8:
        ptc.callPreset("04");
        dirty = true;
        break;
      default:
        dirty = false;
    }

  }
}

