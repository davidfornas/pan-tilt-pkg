#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pan_tilt_camera_teleop/PanTiltController.h>
 
static const std::string OPENCV_WINDOW = "Image window";


class AutoFocus
{
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  image_transport::Publisher image_pub_;
  std::string topic1_, topic2_;
  sensor_msgs::CameraInfo panTiltInfo_;
  
public:
  AutoFocus(ros::NodeHandle &nh, std::string tpc1, std::string tpc2): it_(nh), topic1_(tpc1), topic2_(tpc2)
  {
	ros::NodeHandle n;
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe(topic1_, 1, &AutoFocus::autoFocusCb, this);
    camera_info_sub_ = n.subscribe(topic2_, 1, &AutoFocus::cameraCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~AutoFocus()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
  
  
  void cameraCb(const sensor_msgs::CameraInfo& infoCamera){
	panTiltInfo_ = infoCamera;
	  
	std::cout << panTiltInfo_ << std::endl;
	//sleep(10);
	
  }
  
  
  
  void autoFocusCb(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;	//creo la img
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
	cv::Mat grey;
    cvtColor(cv_ptr->image, grey, CV_BGR2GRAY);	//paso la imagen a escala de grises
	
	cv::Mat imgSobel;
	
	//for (int i = 0/*MinFocusValue*/; i <= 2/*MaxFocusValue*/; i++/*Step*/)
		Sobel(grey, imgSobel, CV_32F, 1, 0, 5);	//5 es el tamaño del filtro
	
	cv::Scalar V = mean(abs(imgSobel));
	//std::cout << V[0] << std::endl;

	cv_ptr->image = imgSobel;
	//imshow("image", imgSobel);
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};



  
int main (int argc, char** argv){
	ros::init(argc, argv, "auto_focus");
	ros::NodeHandle nh;
	AutoFocus af(nh, "/panTilt/image_raw", "/panTilt/camera_info");
	ros::spin();
	  
	return 0;
}
