#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class MyRobot {
private :
  ros::Publisher cmd_pub;
public :
  MyRobot(ros::NodeHandle &n){
    cmd_pub = n.advertise<geometry_msgs::Twist>("/my_robot/vel_cmd", 100);
  }

  // Deplacement avec manette
  void joy_to_twist(const sensor_msgs::Joy &joy){
    geometry_msgs::Twist tw;
    tw.linear.x = 0.15*joy.axes[1];
    tw.angular.z = 2.*joy.axes[0];
    cmd_pub.publish(tw);
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
