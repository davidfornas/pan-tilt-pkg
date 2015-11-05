#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pan_tilt_image_processing/AutoFocusAction.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

#include "std_msgs/Float32MultiArray.h"

#include <time.h>

#define SECONDS 7
//std::vector<cv::Point2f> _scene_corners(4);
uint _direction = 1;
uint _cont = 0;
bool _salir = false, _cambio = false;


class AutoFocusClient{
  protected:
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    std::string topic_;

    actionlib::SimpleActionClient<pan_tilt_image_processing::AutoFocusAction> ac_;

    time_t initTime_, currentTime_;


  public:
    AutoFocusClient (std::string tpc): ac_("auto_focus", true), it_(nh_), topic_(tpc)
    { 
        image_sub_ = it_.subscribe(topic_, 1, &AutoFocusClient::imageCB, this); 
        time(&initTime_);
    }

    void imageCB(const sensor_msgs::ImageConstPtr& msg)
    {
        actionlib::SimpleActionClient<pan_tilt_image_processing::AutoFocusAction> ac("auto_focus", true);

        //ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start

        ac.waitForServer(); //will wait for infinite time

        pan_tilt_image_processing::AutoFocusGoal goal;
        // send a goal to the action
        goal.order = _direction;
        ac.sendGoal(goal);

        if ( (time(&currentTime_) - initTime_ ) > SECONDS ) 
        {
            if (!_cambio)
            {
                _cambio = true;
                initTime_ = currentTime_;
                goal.order = _direction = 2; 
                std::cout << "CAMBIO!" << std::endl;
            }
            else
            { 
                _salir = true;
                _direction = 0;
                exit(EXIT_SUCCESS);
            }
        }
        /*while(!salir)
        {
            ac.waitForServer(); //will wait for infinite time

            ROS_INFO("Action server started, sending goal.");
            // send a goal to the action
            goal.order = _direction;
            ac.sendGoal(goal);

            //wait for the action to return
            bool finished_before_timeout = ac.waitForResult(ros::Duration(26.0));    //6 perhaps good

              
            actionlib::SimpleClientGoalState state = ac.getState();
            if (finished_before_timeout and _direction == 0)
            {
                ROS_INFO("Action finished: %s",state.toString().c_str());
                exit(EXIT_SUCCESS);
                //if (state.toString().c_str() == "SUCCEEDED") exit(0);
            }
            else if(!finished_before_timeout)
            {
                ROS_INFO("TIMEOUT: Object Not Found.");
                std::cout << "GETSTATE: " << state.toString().c_str() << std::endl;
                exit(0);
            }

            if ( (time(&currentTime) - _initTime ) < SECONDS ) 
            {
                if (!cambio)
                {
                    cambio = true;
                    _initTime = currentTime;
                    goal.order = 2;
                }
                else
                { 
                    salir = true;
                    _direction = 0;
                    exit(EXIT_SUCCESS);
                }
            }
        }*/
    }

};


int main (int argc, char **argv)
{
    ros::init(argc, argv, "auto_focus_client");

    //ros::NodeHandle nh;
    //ros::Subscriber points = nh.subscribe("/panTilt/image_raw", 1, paintBox);
    AutoFocusClient ac("/panTilt/image_raw");
    ros::spin();
    //exit
    return 0;
}