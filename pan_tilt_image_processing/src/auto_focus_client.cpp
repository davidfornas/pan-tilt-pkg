#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pan_tilt_image_processing/AutoFocusAction.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

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
    bool init_;
    double maxValue_;

    actionlib::SimpleActionClient<pan_tilt_image_processing::AutoFocusAction> ac_;

    time_t initTime_, currentTime_;


  public:
    AutoFocusClient (std::string tpc): ac_("auto_focus", true), it_(nh_), topic_(tpc)
    { 
        image_sub_ = it_.subscribe(topic_, 1, &AutoFocusClient::imageCB, this); 
        time(&initTime_);
        init_ = true;
        maxValue_ = -1;
    }

    void imageCB(const sensor_msgs::ImageConstPtr& msg)
    {
        actionlib::SimpleActionClient<pan_tilt_image_processing::AutoFocusAction> ac("auto_focus", true);

        //ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start

        ac.waitForServer(); //will wait for infinite time

        pan_tilt_image_processing::AutoFocusGoal goal;
        if (init_)
        {
            // send a goal to the action
            goal.order = _direction;
            ac.sendGoal(goal);
            init_ = false;
        }

        if ( (time(&currentTime_) - initTime_ ) > SECONDS && !_salir) 
        {
            initTime_ = currentTime_;
            if (!_cambio)
            {
                goal.order = 3;     //paro el Iris cuando abre
                ac.sendGoal(goal);

                _cambio = true;
                initTime_ = currentTime_;
                goal.order = _direction = 2; 
                std::cout << "CAMBIO!" << std::endl;
                ac.sendGoal(goal);  //cambio el sentido del Iris
            }
            else
            { 
                goal.order = 4;     //paro el Iris cuando cierra
                ac.sendGoal(goal);

                _salir = true;
                goal.order = _direction = 1;    //volvemos a enfocar al inifinito
                ROS_INFO("The maximum Sobel is %f", maxValue_);
                ac.sendGoal(goal);
                //exit(EXIT_SUCCESS);
            }
        }
        if(!_salir)
        {
            double resSobel = calculateSobel(msg); 
            if ( resSobel > maxValue_){ maxValue_ = resSobel; std::cout << "entra aki?" << std::endl; }
        }
        else
        {   
            goal.order = 3;     //paro el Iris cuando abre
            if((abs(maxValue_ - calculateSobel(msg)) < 0.1) )     //or (time(&currentTime_) - initTime_ ) > SECONDS)
            {
                ac.sendGoal(goal);
                ROS_INFO("FOCUSED %f", maxValue_);            
                goal.order = _direction = 0;
                ac.sendGoal(goal);
                exit(EXIT_SUCCESS);
            }
            else if( (time(&currentTime_) - initTime_ ) > SECONDS )
            {
                ROS_INFO ("DONT FUCUSED :(");
                ac.sendGoal(goal);
                usleep(1500);
                exit(0);
            }
        }
    }

    double calculateSobel(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;   //creo la img
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return -1;
        }
            
        cv::Mat grey;
        cvtColor(cv_ptr->image, grey, CV_BGR2GRAY); //paso la imagen a escala de grises
            
        cv::Mat imgSobel;
            
        //for (int i = 0/*MinFocusValue*/; i <= 2/*MaxFocusValue*/; i++/*Step*/)
        Sobel(grey, imgSobel, CV_32F, 1, 0, 3); //5 es el tamaño del filtro
            
        cv::Scalar V = mean(abs(imgSobel));
        //std::cout << V[0] << std::endl;
        return V[0];
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