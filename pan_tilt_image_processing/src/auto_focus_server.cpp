#include <time.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pan_tilt_image_processing/AutoFocusAction.h>
#include <pan_tilt_camera_teleop/PanTiltController.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#define SECONDS 1

class AutoFocusAction
{
  protected:

  	ros::NodeHandle nh_;
  	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
  	actionlib::SimpleActionServer<pan_tilt_image_processing::AutoFocusAction> as_; 
  	std::string action_name_;
  	// create messages that are used to published feedback/result
  	pan_tilt_image_processing::AutoFocusFeedback feedback_;
  	pan_tilt_image_processing::AutoFocusResult result_;
  	PanTiltController ptc_;

  public:

  	AutoFocusAction(std::string name): as_(nh_, name, boost::bind(&AutoFocusAction::executeCB, this, _1), false), action_name_(name) { as_.start(); }

    ~AutoFocusAction(void){}

  	void executeCB(const pan_tilt_image_processing::AutoFocusGoalConstPtr &goal)
  	{
	    // helper variables
	    ros::Rate r(1);
	    bool success = false;

	    ROS_INFO("Executing the AUTO-FOCUS");

	    switch(goal->order)
	    {
	    	case 1:
	    			ptc_.focusFarStart();
	    			ROS_INFO("Executing focus FAR");
	      			success = true;
	    		break;
	    	case 2:
	    			ptc_.focusNearStart(); 
	    			ROS_INFO("Executing focus CLOSE");
	      			success = true;
	    		break;
	    	case 3:
	    			ptc_.focusFarStop(); 
	    			ROS_INFO("STOPING FOCUS FAR");
	      			success = true;
	    		break;
	    	case 4:
	    			ptc_.focusNearStop(); 
	    			ROS_INFO("STOPING FOCUS CLOSE");
	      			success = true;
	    		break;
	    	case 0:	
	    			ptc_.focusFarStop();
	    			ROS_INFO("STOP");
	      			success = true;
	    		break;
	    }

	    if(success)
	    {
	      result_.mean_abs_sobel = feedback_.realized = 0;
	      if(goal->order == 1) ROS_INFO("%s: Succeeded", action_name_.c_str());
	      if(goal->order == 0) ROS_INFO("Focused!!");
	      // set the action state to succeeded
	      as_.setSucceeded(result_);
	    }
  	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "auto_focus");

	AutoFocusAction findObject(ros::this_node::getName());
	ros::spin();

	return 0;
}