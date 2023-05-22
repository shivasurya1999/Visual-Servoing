// header for ROS core functionalities
#include "rclcpp/rclcpp.hpp"

// including image message type to be able to receive and publish it
#include "sensor_msgs/msg/image.hpp"

// headers regarding the connection between opencv and ROS
#include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"

// OpenCV core functions and various image processing algorithms
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using std::placeholders::_1;

// Defining a class that will be utilize in the "main" function
class ImageSubscriber : public rclcpp::Node
{

	// Declaring pointer to the publisher and subscriber the publish and receive images.
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  public:
	// Constructor of the class. The class is derived from the parent class "Node" of rclcpp and
	// the node is called "image_processor", and each time it receive a data, it will call the callbackImage() function
    	ImageSubscriber() : Node("image_processor")
	{
		// Defining the subscriber: it will receive "Image" type data from the topic /camera1/image_raw 
      		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, std::bind(&ImageSubscriber::callbackImage, this, _1));

	//defining the publisher: it will publish "Image" type data to the "output_image" topic
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
    }

  private:

	// callback function which will be triggered each time the subscriber_ receives new data.
	// The data it receives is of Image type.
	void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg) 
	{    
		//converting the ROS message type data to opencv type
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg,"bgr8" );
		
		cv::Mat threshold_img = cv_ptr->image;
		cv::Mat hsv_img;
		cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);
		cv::Mat green_;
		cv::inRange(hsv_img, cv::Scalar(50, 100, 100), cv::Scalar(95, 255, 255), green_);
		
		cv::Mat red_;
		cv::inRange(hsv_img, cv::Scalar(0, 65, 100), cv::Scalar(20, 255, 255), red_);
		
		cv::Mat magenta_;
		cv::inRange(hsv_img, cv::Scalar(135, 90, 90), cv::Scalar(150, 255, 255), magenta_);
		
		cv::Mat blue_;
		cv::inRange(hsv_img, cv::Scalar(95, 95, 95), cv::Scalar(125, 255, 255), blue_);

		cv::Point blue_center = cal_center(blue_);
		cv::Point red_center = cal_center(red_);
		cv::Point green_center = cal_center(green_);
		cv::Point magenta_center = cal_center(magenta_);

		cv::circle(threshold_img, blue_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		cv::circle(threshold_img, green_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		cv::circle(threshold_img, red_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		cv::circle(threshold_img, magenta_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		
		sensor_msgs::msg::Image::SharedPtr msg_to_send = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",threshold_img).toImageMsg();
		// converting opencv data type to ROS message type
   		// auto msg_to_send = cv_ptr->toImageMsg();

		// publishing the image.
   		publisher_->publish(*msg_to_send);
    }
	cv::Point cal_center(cv::Mat mask)
    {
		long xAvg = 0, yAvg = 0, count = 0;
		for (int i = 0; i < mask.rows; i++)
		{
			for (int j = 0; j < mask.cols; j++)
			{
				if((int)mask.at<uchar>(i,j) == 255)
				{
					xAvg += j;
					yAvg += i;
					count += 1;
				}
			}
			
		}
		cv::Point center = cv::Point(int(xAvg/count), int(yAvg/count));
		return center;	
	}
    
};

int main(int argc, char * argv[])
{

	//initialize ROS
	rclcpp::init(argc, argv);

	//create the 
	rclcpp::spin(std::make_shared<ImageSubscriber>());
	rclcpp::shutdown();
  return 0;
}