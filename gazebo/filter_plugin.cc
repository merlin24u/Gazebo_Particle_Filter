#ifndef _FILTER_PLUGIN_HH_
#define _FILTER_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <vector>
#include <math.h>
#include <random>
#include "../src/desc_robot.hpp"

using namespace std;

int N = 10; // number of particles used by default
volatile int P = 0; // index of current particle in simulation

struct Particle{
  float posX, posY, angle, density; // coordinates and orientation of a particle
  float obs; // camera_depth information
  gazebo::common::Time stamp;
  Particle(float x, float y, float alpha, gazebo::common::Time t) : posX(x), posY(y), angle(alpha), stamp(t) {
    obs = 0.0;
    density = 1.0;
  } 
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

    /// \brief A ROS subscriber for cmd_vel topic
    ros::Subscriber rosSub_cmd;

    /// \brief A ROS subscriber for camera_depth topic
    ros::Subscriber rosSub_cam;

    /// \brief A ROS subscriber for camera_depth topic of particles
    ros::Subscriber rosSub_cam_filter;

    /// \brief A ROS callbackqueue that helps process messages concerning cmd_vel info
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue concerning cmd_vel info
    thread rosQueueThread;

    /// \brief A ROS callbackqueue that helps process messages concerning camera info
    ros::CallbackQueue rosQueue_cam;

    /// \brief A thread the keeps running the rosQueue concerning camera info
    thread rosQueueThread_cam;
  
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

      // Store the world pointer for convenience.
      this->world = this->model->GetWorld();

      // Check that sdf elements exist, then read the values
      if (_sdf->HasElement("velocity"))
	MAX_SPEED = _sdf->Get<double>("velocity");
      if (_sdf->HasElement("nb_particles"))
	N = _sdf->Get<double>("nb_particles");

      // Random initialization of N particles
      random_device rd;
      default_random_engine generator(rd());
      uniform_real_distribution<float> distribution_width(-MAX_WIDTH + SIZE_ROBOT, MAX_WIDTH - SIZE_ROBOT);
      uniform_real_distribution<float> distribution_height(-MAX_HEIGHT + SIZE_ROBOT, MAX_HEIGHT - SIZE_ROBOT);
      uniform_real_distribution<float> distribution_angle(0, 6.28319);
      gazebo::common::Time stamp = gazebo::common::Time::GetWallTime();

      for(int i = 0; i < N; i++){
	float posX = distribution_width(generator);
	float posY = distribution_height(generator);
	float angle = distribution_angle(generator);
	
	Particle p(posX, posY, angle, stamp);
	particles.push_back(p);
      }

      // Initialize ros, if it has not already bee initialized.
      if(!ros::isInitialized()){
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "gazebo",
		  ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_filter_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so_cmd =
	ros::SubscribeOptions::create<geometry_msgs::Twist>(
							    "/my_robot/vel_cmd",
							    1,
							    boost::bind(&FilterPlugin::on_cmd, this, _1),
							    ros::VoidPtr(), &this->rosQueue);

      this->rosSub_cmd = this->rosNode->subscribe(so_cmd);

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so_cam =
	ros::SubscribeOptions::create<std_msgs::Float32>(
							 "/my_robot/camera",
							 100,
							 boost::bind(&FilterPlugin::on_camera, this, _1),
							 ros::VoidPtr(), &this->rosQueue_cam);
      
      this->rosSub_cam = this->rosNode->subscribe(so_cam);

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so_cam_filter =
	ros::SubscribeOptions::create<std_msgs::Float32>(
							 "/filter/camera",
							 100,
							 boost::bind(&FilterPlugin::on_camera_filter, this, _1),
							 ros::VoidPtr(), &this->rosQueue_cam);
      
      this->rosSub_cam_filter = this->rosNode->subscribe(so_cam_filter);

      // Spin up the queue helper thread.
      this->rosQueueThread =
	thread(bind(&FilterPlugin::QueueThread, this));

      // Spin up the queue helper thread.
      this->rosQueueThread_cam =
	thread(bind(&FilterPlugin::QueueThread_cam, this));
    }

    /// \brief Handle an incoming message from ROS cmd_vel
    /// \param[in] data Joy inputs that is used to set the velocity
    /// of all particles.
    void on_cmd(const geometry_msgs::TwistConstPtr &data)
    {
      float x = -data->linear.x * 1000.0; // in millimeters
      float th = data->angular.z * (BASE_WIDTH/2);
      float k = max(abs(x - th), abs(x + th));
      
      // sending commands higher than max speed will fail
      if (k > MAX_SPEED){
	x = x * MAX_SPEED / k;
	th = th * MAX_SPEED / k;
      }

      gazebo::common::Time now = gazebo::common::Time::GetWallTime();
      for(int i = 0; i < N; i++)
	setPose(i, now, x - th, x + th);
    }

    /// \brief Handle an incoming message from ROS camera_depth
    /// \param[in] msg ROS camera_depth data
    void on_camera(const std_msgs::Float32ConstPtr &msg)
    {
      // cout << msg->data << endl;
      for(int i = 0; i < N; i++)
	cout << particles[i].obs << " ";
      cout << endl;
    }

    /// \brief Handle an incoming message from ROS camera_depth particle
    /// \param[in] msg ROS camera_depth data
    void on_camera_filter(const std_msgs::Float32ConstPtr &msg)
    {
      // cout << P << endl;
      particles[P].obs = msg->data;
    }

    /// \brief Set the new pose of a particle
    /// \param[in] t Time cmd_vel msg has been sent 
    /// \param[in] p Particle
    /// \param[in] r Right target velocity
    /// \param[in] l Left target velocity
    void setPose(const int &p, const gazebo::common::Time &t, const double &l, const double &r)
    {
      P = p; // set the current particle in simulation
      float dt = (t - particles[p].stamp).Float() * 1000; // in ms
      particles[p].stamp = gazebo::common::Time::GetWallTime();
      
      ignition::math::Pose3d initPose(ignition::math::Vector3d(particles[p].posX, particles[p].posY, 0.), ignition::math::Quaterniond(0., 0., particles[p].angle));
      this->model->SetWorldPose(initPose);

      this->model->GetJoint("left_wheel_hinge")->SetVelocity(0, l);
      this->model->GetJoint("right_wheel_hinge")->SetVelocity(0, r);

      this->world->Step(500); // nb of iterations
      
      auto model_pose = this->model->WorldPose();
      auto pose = model_pose.Pos();
      auto rot = model_pose.Rot();
      particles[p].posX = pose.X();
      particles[p].posY = pose.Y();
      particles[p].angle = rot.Yaw();
    }

    // Cumulative distribution function of normal distribution
    float cdf(float x, float mean, float std_dev){
      return 0.5 * erfc(-(x - mean) / (std_dev * sqrt(2)));
    }

    // Probability density function of normal distribution
    float pdf(float x, float mean, float std_dev){
      return 1 / (std_dev * sqrt(2*M_PI)) * exp(-pow(x - mean, 2) / (2 * pow(std_dev, 2)));
    }

  private:
    /// \brief ROS helper function that processes messages
    void QueueThread()
    {
      static const double timeout = 0.01;
      while(this->rosNode->ok()){
	this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// \brief ROS helper function that processes messages
    void QueueThread_cam()
    {
      static const double timeout = 0.01;
      while(this->rosNode->ok()){
	this->rosQueue_cam.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(FilterPlugin)
}
#endif
