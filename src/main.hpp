#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <gazebo_particle_filter/Sensor.h>

class MyRobot{
private:
  ros::Publisher cmd_pub, camera_pub, camera_filter_pub;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub, image_filter_sub;
  
public :
  // Robot characteristics
  const static float BASE_WIDTH; // in millimeters
  const static float WHEEL_RADIUS; // in meters
  const static float MAX_WIDTH; // width limit of map in meters
  const static float MAX_HEIGHT; // height limit of map in meters
  const static float SIZE_ROBOT; // size max of robot in meters
  static float MAX_SPEED; // speed in radian/s of wheels

  // Camera description
  const static float STD_DEV; // std of sensor
  const static float V_MIN; // value min
  const static float V_MAX; // value max
  
  MyRobot(ros::NodeHandle &n);
  void joy_to_twist(const sensor_msgs::Joy &joy);
  void imagePub(const sensor_msgs::ImageConstPtr &img);
  void imageFilterPub(const sensor_msgs::ImageConstPtr &img);
  int imageCb(const sensor_msgs::ImageConstPtr &img, gazebo_particle_filter::Sensor &msg);
};
