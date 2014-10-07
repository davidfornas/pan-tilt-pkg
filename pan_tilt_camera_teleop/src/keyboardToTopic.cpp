#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
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
        sensor_msgs::JointState js;
        ros::Publisher pub_;

    public:
        PanTiltKeyboardTeleopNode()
        {
            pub_ = n_.advertise<sensor_msgs::JointState>("panTilt", 1);
            
            ros::NodeHandle n_private("~");
            //n_private.param("walk_vel", walk_vel_, 0.5);
         //   n_private.param("run_vel", run_vel_, 1.0);
         //   n_private.param("yaw_rate", yaw_rate_, 1.0);
         //   n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
            js.name.push_back("pan");
            js.name.push_back("tilt");
            js.position.push_back(0.0);
            js.position.push_back(0.0);

        }
        
        ~PanTiltKeyboardTeleopNode() { }
        void keyboardLoop();
        void stopCamera(){
          //js.position[0]=0.0;
          //js.position[1]=0.0;
          //pub_.publish(js);
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
                js.position[0]=0.0;
                  js.position[1]=1.0;
                  pub_.publish(js);
                dirty = true;
                break;
            case KEYCODE_S:
                js.position[0]=0.0;
                  js.position[1]=-1.0;
                  pub_.publish(js);
                dirty = true;
                break;
            case KEYCODE_A:
                js.position[0]=1.0;
                  js.position[1]=0.0;
                  pub_.publish(js);
                dirty = true;
                break;
            case KEYCODE_D:
                js.position[0]=-1.0;
                  js.position[1]=0.0;
                  pub_.publish(js);
                dirty = true;
                break;
            //case KEYCODE_W_CAP:
            //    dirty = true;
            //    break;
            //case KEYCODE_Z:
            //    ptc.teleStart();
            //    dirty = true;
            //    break;
            //case KEYCODE_X:
            //    ptc.wideStart();
            //    dirty = true;
            //    break;
            //case KEYCODE_Q:
            //    ptc.focusNearStart();
            //    dirty = true;
            //    break;
            //case KEYCODE_E:
            //    ptc.focusFarStart();
            //    dirty = true;
            //    break;
            //case KEYCODE_1:
            //    ptc.setPreset("01");
            //    dirty = true;
            //    break;
            //case KEYCODE_2:
            //    ptc.setPreset("02");
            //    dirty = true;
            //    break;
            //case KEYCODE_3:
            //    ptc.setPreset("03");
            //    dirty = true;
            //    break;
            //case KEYCODE_4:
            //    ptc.setPreset("04");
            //    dirty = true;
            //    break;
            //case KEYCODE_5:
            //    ptc.callPreset("01");
            //    dirty = true;
            //    break;
            //case KEYCODE_6:
            //    ptc.callPreset("02");
            //    dirty = true;
            //    break;
            //case KEYCODE_7:
            //    ptc.callPreset("03");
            //    dirty = true;
            //    break;
            //case KEYCODE_8:
            //    ptc.callPreset("04");
            //    dirty = true;
            //    break;
            default:
                dirty = false;
        }
        
    }
}

