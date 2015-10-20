#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pan_tilt_camera_teleop/PanTiltController.h>
 
static const std::string OPENCV_WINDOW = "Image Pan-Tilt";


PanTiltController ptc;
uint n = 153333;  //milisegundos
bool bandera = false;
bool _star = false;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter(): it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/panTilt/image_raw", 1, &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;	//creo la img
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Scalar tempVal= mean(cv_ptr->image);
   if(tempVal[0] > 135 and !_star){
		ptc.irisCloseStart();
		bandera = true;
		_star = true;
	}
	else if(tempVal[0] < 75 && !bandera and !_star){
		ptc.irisOpenStart();
		_star = true;
		//usleep(n);
		//ptc.irisOpenStop();
	}
	else{
		usleep(n);			//para poder hacer pausas inferiores al segundo (milisegundos)
		if(bandera)
			ptc.irisCloseStop();
		else
			ptc.irisOpenStop();
		_star = false;
	}
	usleep(n);
    std::cout << tempVal[0] << std::endl;
    bandera = false;
  }
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(1);
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  
  return 0;
}
