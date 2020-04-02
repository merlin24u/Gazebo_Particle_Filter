#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class MyRobot {
private :
  ros::Publisher cmd_pub, camera_pub;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  
public :
  MyRobot(ros::NodeHandle &n) : it(n) {
    cmd_pub = n.advertise<geometry_msgs::Twist>("/my_robot/vel_cmd", 100);
    camera_pub = n.advertise<std_msgs::Float32>("/my_robot/camera", 100);

    // Subscrive to input video feed
    image_sub = it.subscribe("/camera/depth/image_raw", 1,
			     &MyRobot::imageCb, this);
  }

  // Deplacement avec manette
  void joy_to_twist(const sensor_msgs::Joy &joy){
    geometry_msgs::Twist tw;
    tw.linear.x = 0.1 * joy.axes[1];
    tw.angular.z = 0.5 * joy.axes[0];
    cmd_pub.publish(tw);
  }

  // Conversion Msg -> openCV image
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
    }catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int r = cv_ptr->image.rows;
    int c = cv_ptr->image.cols;

    float depth = cv_ptr->image.at<float>(r/2, c/2);

    if(!std::isnan(depth)){
      std_msgs::Float32 msg;
      msg.data = depth;
      
      camera_pub.publish(msg);
    }
  }
};
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot");
  ros::NodeHandle n;
  MyRobot r(n);

  ros::Subscriber joy_sub = n.subscribe("/joy", 100, &MyRobot::joy_to_twist, &r);
  ros::spin();
 
  return 0;
}
