#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ross::Publisher vel_pub_;

  //angular and linear vels, ball pos
  int angular, linear;
  int ball_row, ball_col;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    // sub to depth feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, 
		    &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("camera/depth/image", 10,
		    &Follow::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);
    

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int row = 0; 
    int col = 0; 
    int dis_min = 1000000; 
    for (int i=0; i < cv_ptr->image.rows; i++) {
        for (int j=0; j < cv_ptr->image.cols; j++) {
            int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0];
            int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
            int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
            // int dis = pow((r-255)*(r-255) + g*g + b*b, 0.5); 
            int dis = (r-255)*(r-255) + g*g + b*b; 
            // ROS_INFO("b: %d", b); 
            // ROS_INFO("g: %d", g); 
            // ROS_INFO("r: %d", r); 
            // ROS_INFO("%d", dis); 
            if (dis < dis_min) {
                dis_min = dis; 
                row = i; 
                col = j; 
            }
        }
    }

    ball_row = row;
    ball_col = col;

    if(ball_col < msg->width * 0.45)
	    angular = 1;
    else if(ball_col > msg->widht * 0.55)
	    angular = -1;
    else
	    ;

    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[0]); 
    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[1]); 
    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[2]); 
    // ROS_INFO("height: %d", msg->height); 
    // ROS_INFO("width: %d", msg->width); 
    // ROS_INFO_STREAM("encoding: " << msg->encoding); 

   
    // ROS_INFO("r: %d", row); 
    // ROS_INFO("c: %d", col); 

    cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void depthCb(const sensor_msgs::Image::ConstPtr& msg)
  {
	  //get dist to ball
	  cv_bridge::CvImagePtr cv_ptr;
	  try
	  {
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		  float distance = cv_ptr->image.at<float>(ball_row, ball_col);
		  ROS_INFO("Distance: %f", distance);

		  if (distance < 1.0)
			  linear = -1;
		  else if (distance > 1.1)
			  linear = 1;
		  else
			  ;

		  move();
	  }
	  catch (cv_bridge::Exception& e)
	  {
		  ROS_ERROR("cv_bridge exception: %s", e.what());
	  }
  }

  void move()
  {
	  float lspd = 0.125;
	  float apd = 0.250;

	  geometry_msgs::Twist vel_msg;

	  vel_msg.linear.y = vel_msg.linear.z = 0;
	  vel_msg.angular.x = vel_msg.angular.y = 0;
	  vel_msg.linear.x = lspd * linear; 
	  vel_msg.angular.z = aspd * angular;

	  vel_pub_.publish(vel_msg);
  }

}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
