#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <my_robot/Obs.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/core/mat.hpp>
#include "desc_robot.hpp"

class MyRobot {
private :
  ros::Publisher cmd_pub, camera_pub, camera_filter_pub;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub, image_filter_sub;
  
public :
  MyRobot(ros::NodeHandle &n) : it(n) {
    cmd_pub = n.advertise<geometry_msgs::Twist>("/my_robot/vel_cmd", 100);
    camera_pub = n.advertise<my_robot::Obs>("/my_robot/camera", 100);
    camera_filter_pub = n.advertise<my_robot::Obs>("/filter/camera", 100);

    // Subscrive to input video feed
    image_sub = it.subscribe("/camera/depth/image_raw", 100,
			     &MyRobot::imagePub, this);

    // Subscrive to input video feed from particles
    image_filter_sub = it.subscribe("/camera_filter/depth/image_raw", 100,
				    &MyRobot::imageFilterPub, this);
  }

  // Deplacement avec manette
  void joy_to_twist(const sensor_msgs::Joy &joy){
    geometry_msgs::Twist tw;
    tw.linear.x = 0.1 * joy.axes[1];
    tw.angular.z = 0.5 * joy.axes[0];
    cmd_pub.publish(tw);
  }

  void imagePub(const sensor_msgs::ImageConstPtr &img){
    my_robot::Obs msg;
    int res = imageCb(img, msg);

    if(res != -1)
      camera_pub.publish(msg);
  }

  void imageFilterPub(const sensor_msgs::ImageConstPtr &img){
    my_robot::Obs msg;
    int res = imageCb(img, msg);

    if(res != -1)
      camera_filter_pub.publish(msg);
  }

  // Conversion Msg -> openCV image
  int imageCb(const sensor_msgs::ImageConstPtr& img, my_robot::Obs &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(img, "32FC1");
    }catch(const cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return -1;
    }

    cv::Mat depth_img = cv_ptr->image;
    cv::Mat noise = cv::Mat(depth_img.size(), CV_32F);
    cv::randn(noise, 0.0, STD_DEV);
    depth_img += noise;
    
    int r = depth_img.rows;
    int c = depth_img.cols;

    msg.data.push_back(depth_img.at<float>(r/2, c/2));
    msg.data.push_back(depth_img.at<float>(r/2, 0));
    msg.data.push_back(depth_img.at<float>(r/2, c - 1));

    for(int i = 0; i < msg.data.size(); i++)
      if(std::isnan(msg.data[i]))
	return -1;

    return 0;
  }
};
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot");
  ros::NodeHandle n;
  MyRobot r(n);

  cv::theRNG().state = cv::getTickCount(); // set seed for rng noise
  
  ros::Subscriber joy_sub = n.subscribe("/joy", 100, &MyRobot::joy_to_twist, &r);
  ros::spin();
 
  return 0;
}
