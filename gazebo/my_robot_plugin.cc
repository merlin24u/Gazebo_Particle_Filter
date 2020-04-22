#ifndef _MYROBOT_PLUGIN_HH_
#define _MYROBOT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <vector>
#include <math.h>
#include "../src/desc_robot.hpp"

using namespace std;

namespace gazebo
{
  /// \brief A plugin to control a MyRobot sensor.
  class MyRobotPlugin : public ModelPlugin
  {

  private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;
    
    /// \brief A node use for ROS transport
    unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    thread rosQueueThread;
  
  public:
    /// \brief Constructor
    MyRobotPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      // Safety check
      if(_model->GetJointCount() == 0){
	cerr << "Invalid joint count, MyRobot plugin not loaded\n";
	return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
	MAX_SPEED = _sdf->Get<double>("velocity");

      // Initialize ros, if it has not already bee initialized.
      if(!ros::isInitialized()){
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "gazebo",
		  ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
	ros::SubscribeOptions::create<geometry_msgs::Twist>(
							    "/my_robot/vel_cmd",
							    100,
							    boost::bind(&MyRobotPlugin::onRosMsg, this, _1),
							    ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
	thread(bind(&MyRobotPlugin::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] data Joy inputs that is used to set the velocity
    /// of the MyRobot.
    void onRosMsg(const geometry_msgs::TwistConstPtr &data)
    {
      float x = data->linear.x * 1000.0;
      float th = data->angular.z * (BASE_WIDTH/2);
      float k = max(abs(x - th), abs(x + th));
      
      // scale cmd_vel with max speed
      if (k > MAX_SPEED){
	x = x * MAX_SPEED / k;
	th = th * MAX_SPEED / k;
      }

      if(x >= 0)
	this->setVelocity(x - th, x + th);
      else
	this->setVelocity(x + th, x - th);
    }

    /// \brief Set the velocity of the MyRobot
    /// \param[in] l New left target velocity
    /// \param[in] r New right target velocity
    void setVelocity(const double &l, const double &r)
    {
      this->model->GetJoint("my_robot::left_wheel_hinge")->SetVelocity(0, l);
      this->model->GetJoint("my_robot::right_wheel_hinge")->SetVelocity(0, r);
    }

  private:
    /// \brief ROS helper function that processes messages
    void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
	{
	  this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MyRobotPlugin)
}
#endif
