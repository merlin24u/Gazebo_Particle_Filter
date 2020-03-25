#ifndef _MYROBOT_PLUGIN_HH_
#define _MYROBOT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"

using namespace std;

const int BASE_WIDTH = 200;    // millimeters
int MAX_SPEED = 10;     // m/s

namespace gazebo
{
  /// \brief A plugin to control a MyRobot sensor.
  class MyRobotPlugin : public ModelPlugin
  {

  private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
    vector<physics::JointPtr> joint;
    
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

      this->joint = vector<physics::JointPtr>(begin(_model->GetJoints()), end(_model->GetJoints()));

      // Init PID
      /*
	common::PID pid(0.1, 0, 0);
	this->model->GetJointController()->SetVelocityPID(this->joint[0]->GetScopedName(), pid);
	this->model->GetJointController()->SetVelocityPID(this->joint[0]->GetScopedName(), pid);
      */

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
	MAX_SPEED = _sdf->Get<double>("velocity");

      // Initialize ros, if it has not already bee initialized.
      if(!ros::isInitialized()){
	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, "gazebo_client",
		    ros::init_options::NoSigintHandler);
	}

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
	ros::SubscribeOptions::create<geometry_msgs::Twist>(
							 "/" + this->model->GetName() + "/vel_cmd",
							 1,
							 boost::bind(&MyRobotPlugin::OnRosMsg, this, _1),
							 ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
	thread(bind(&MyRobotPlugin::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] data Joy inputs that is used to set the velocity
    /// of the MyRobot.
    void OnRosMsg(const geometry_msgs::TwistConstPtr &data)
    {
      float x = -data->linear.x * 1000.0; // from meters to millimeters
      float th = data->angular.z * (BASE_WIDTH/2); // in mm
      float k = max(abs(x - th), abs(x + th));
      
      // sending commands higher than max speed will fail
      if (k > MAX_SPEED){
	x = x * MAX_SPEED / k;
	th = th * MAX_SPEED / k;
      }
      
      this->SetVelocity(x - th, x + th);
    }

    /// \brief Set the velocity of the MyRobot
    /// \param[in] _x New target velocity
    /// \param[in] _y New target velocity
    void SetVelocity(const double &_x, const double &_y)
    {
      // cout << "right : " << _x << ", left : " << _y << endl;

      // Set the joint's target velocity.
      this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, _x);
      this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, _y);

      /*
      this->model->GetJointController()->SetVelocityTarget(this->joint[0]->GetScopedName(), _x);
      this->model->GetJointController()->SetVelocityTarget(this->joint[1]->GetScopedName(), _y);
      */
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
