#include <time.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pan_tilt_object_localization/FindAnObjectAction.h>
#include <pan_tilt_camera_teleop/PanTiltController.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#define SECONDS 1

class FindAnObjectAction
{
  protected:

  	ros::NodeHandle nh_;
  	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
  	actionlib::SimpleActionServer<pan_tilt_object_localization::FindAnObjectAction> as_; 
  	std::string action_name_;
  	// create messages that are used to published feedback/result
  	pan_tilt_object_localization::FindAnObjectFeedback feedback_;
  	pan_tilt_object_localization::FindAnObjectResult result_;
  	PanTiltController ptc_;

  public:

  	FindAnObjectAction(std::string name): as_(nh_, name, boost::bind(&FindAnObjectAction::executeCB, this, _1), false), action_name_(name) { as_.start(); }

    ~FindAnObjectAction(void){}

	void turnCam()
	{
	  	time_t initTime, current;
	  	time(&initTime);

		while ((time(&current) - initTime) < SECONDS)
		{
	  		ptc_.left();
	    	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	    	ptc_.stopPanTilt();
	    }
	}

  	void executeCB(const pan_tilt_object_localization::FindAnObjectGoalConstPtr &goal)
  	{
	    // helper variables
	    ros::Rate r(1);
	    bool success = false;

	    // push_back the seeds for the fibonacci sequence
	    /*feedback_.sequence.clear();
	    feedback_.sequence.push_back(0);
	    feedback_.sequence.push_back(1);*/

	    // publish info to the console for the user
	    //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->direction, feedback_.sequence[0], feedback_.sequence[1]);
	    ROS_INFO("Executing the object finder");


	    std::cout << "GOAL = : " << goal->direction << std::endl;

	    if(goal->direction == 1)
	    {
	    	turnCam();
	    	//boost::thread turn_cam(&turnCam);
	      	//turn_cam.join();
	      	success = true;
	    } //td::cout << "Izquierda" << std::endl;
	    else success = true;

	    if(success)
	    {
	      result_.sequence = feedback_.sequence = 0;
	      if(goal->direction == 1) ROS_INFO("%s: Succeeded", action_name_.c_str());
	      if(goal->direction == 0) ROS_INFO("Object Found!!");
	      // set the action state to succeeded
	      as_.setSucceeded(result_);
	    }
  	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "find_object");

	FindAnObjectAction findObject(ros::this_node::getName());
	ros::spin();

	return 0;
}