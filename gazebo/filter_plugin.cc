#ifndef _FILTER_PLUGIN_HH_
#define _FILTER_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <vector>
#include <math.h>
#include <random>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"

using namespace std;

const int BASE_WIDTH = 200; // millimeters
const int WHEEL_RADIUS = 100; // millimeters
const float MAX_WIDTH = 5.; // limit map in meter
const float MAX_HEIGHT = 3.25; // limit map in meter
int MAX_SPEED = 10; // radian/s default
int N = 10; // number of particles used
float SIZE_ROBOT = 0.4; // max size of robot in meters

struct Particle{
  float posX, posY, angle, density; // coordinates and orientation of a particle
  gazebo::common::Time stamp;
  Particle(float x, float y, float alpha, float prob, gazebo::common::Time t) : posX(x), posY(y), angle(alpha), density(prob), stamp(t) {} 
};

namespace gazebo
{
  /// \brief A plugin to control a Filter sensor.
  class FilterPlugin : public ModelPlugin
  {

  private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointer to the world.
    physics::WorldPtr world;

    /// \brief An array of particles
    vector<Particle> particles;
    
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
    FilterPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      // Safety check
      if(_model->GetJointCount() == 0){
	cerr << "Invalid joint count, Filter plugin not loaded\n";
	return;
      }
      
      // Store the model pointer for convenience.
      this->model = _model;

      // Store the model pointer for convenience.
      this->world = this->model->GetWorld();

      // Check that sdf elements exist, then read the values
      if (_sdf->HasElement("velocity"))
	MAX_SPEED = _sdf->Get<double>("velocity");
      if (_sdf->HasElement("nb_particles"))
	N = _sdf->Get<double>("nb_particles");
      if (_sdf->HasElement("size_robot"))
	SIZE_ROBOT= _sdf->Get<double>("size_robot");

      // Random initialization of N particles
      random_device rd;
      default_random_engine generator(rd());
      uniform_real_distribution<float> distribution_width(-MAX_WIDTH + SIZE_ROBOT, MAX_WIDTH - SIZE_ROBOT);
      uniform_real_distribution<float> distribution_height(-MAX_HEIGHT + SIZE_ROBOT, MAX_HEIGHT - SIZE_ROBOT);
      uniform_real_distribution<float> distribution_angle(0, 6.28319);
      gazebo::common::Time stamp = this->world->SimTime();

      for(int i = 0; i < N; i++){
	float posX = distribution_width(generator);
	float posY = distribution_height(generator);
	float angle = distribution_angle(generator);
	
	Particle p(posX, posY, angle, 1., stamp);
	particles.push_back(p);
      }

      // Initialize ros, if it has not already bee initialized.
      if(!ros::isInitialized()){
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "gazebo_filter_client",
		  ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_filter_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
	ros::SubscribeOptions::create<geometry_msgs::Twist>(
							    "/my_robot/vel_cmd",
							    100,
							    boost::bind(&FilterPlugin::OnRosMsg, this, _1),
							    ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
	thread(bind(&FilterPlugin::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] data Joy inputs that is used to set the velocity
    /// of the Filter.
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

      if((x - th) != 0 || (x + th) != 0)
	for(int i = 0; i < N; i++)
	  this->SetVelocity(i, x - th, x + th);
    }

    /// \brief Set the velocity of the Filter
    /// \param[in] p particle
    /// \param[in] r New right target velocity
    /// \param[in] l New left target velocity
    void SetVelocity(const int &p, const double &l, const double &r)
    {
      gazebo::common::Time now = this->world->SimTime();
      float dt = (now - particles[p].stamp).Float() * 1000; // in ms
      particles[p].stamp = now;
      
      ignition::math::Pose3d initPose(ignition::math::Vector3d(particles[p].posX, particles[p].posY, 0.), ignition::math::Quaterniond(0., 0., particles[p].angle));
      this->model->SetWorldPose(initPose);

      this->world->SetPaused(false);
      
      // Set the joint's target velocity.
      this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, l);
      this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, r);

      gazebo::common::Time::MSleep(round(dt));

      auto model_pose = this->model->WorldPose();
      auto pose = model_pose.Pos();
      auto rot = model_pose.Rot();
      particles[p].posX = pose.X();
      particles[p].posY = pose.Y();
      particles[p].angle = rot.Yaw();

      world->SetPaused(true);
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
  GZ_REGISTER_MODEL_PLUGIN(FilterPlugin)
}
#endif
