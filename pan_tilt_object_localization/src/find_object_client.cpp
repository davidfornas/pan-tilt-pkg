#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pan_tilt_object_localization/FindAnObjectAction.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

#include "std_msgs/Float32MultiArray.h"

std::vector<cv::Point2f> _scene_corners(4);
uint _direction = 1;
uint _cont = 0;


bool dimmensionBox() { 
    if(abs(_scene_corners[0].x - _scene_corners[1].x) * abs(_scene_corners[1].y - _scene_corners[2].y) > 800) 
    { 
        _direction = 0; 
        return true;
    } 
    else return false;
}

void paintBox(const std_msgs::Float32MultiArray& points)
{
    std::cout << "hola" << std::endl;

    actionlib::SimpleActionClient<pan_tilt_object_localization::FindAnObjectAction> ac("find_object", true);

    for (int i = 0; i < 4; i++)
    {
        _scene_corners[i].x = points.data[i*2];
        _scene_corners[i].y = points.data[i*2+1];
    }
    dimmensionBox();

    //ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    if(_cont++ % 5 == 0)
    {
        ac.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        pan_tilt_object_localization::FindAnObjectGoal goal;
        goal.direction = _direction;
        ac.sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(2.0));    //5 is good

          
        actionlib::SimpleClientGoalState state = ac.getState();
        if (finished_before_timeout)
        {
            ROS_INFO("Action finished: %s",state.toString().c_str());
            if (_direction == 0) exit(0);
            //if (state.toString().c_str() == "SUCCEEDED") exit(0);
        }
        else
        {
            ROS_INFO("Object Not Found.");
            std::cout << "GETSTATE: " << state.toString().c_str() << std::endl;
            exit(0);
        }
    }
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "find_object_client");

    ros::NodeHandle nh;
    ros::Subscriber points = nh.subscribe("/surf/corners_object", 1, paintBox);

    ros::spin();
    //exit
    return 0;
}