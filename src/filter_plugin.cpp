#ifndef _FILTER_PLUGIN_HH_
#define _FILTER_PLUGIN_HH_

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_particle_filter/Sensor.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <vector>
#include <math.h>
#include <random>
#include <algorithm>
#include <limits>
#include "main.hpp"

using namespace std;

struct Particle{
  float posX, posY, angle, weight; // coordinates, orientation and weight of a particle
  vector<float> obs; // camera_depth information

  // Visual info
  ignition::math::Color color;
  string color_name;

  Particle(float x, float y, float alpha, int color_idx, int nbp, int nbs) : posX(x), posY(y), angle(alpha), obs(nbs, 0.0){
    weight = 1.0 / nbp;
    setColor(color_idx);
  }

  Particle(const Particle &p, int color_idx, int nbp) : posX(p.posX), posY(p.posY), angle(p.angle), obs(p.obs){
    weight = 1.0 / nbp;
    setColor(color_idx);
  }

  void setColor(int color_idx){
    switch(color_idx){
    case 0:
      color_name = "Black";
      color = ignition::math::Color::Black;
      break;
    case 1:
      color_name = "Blue";
      color = ignition::math::Color::Blue;
      break;
    case 2:
      color_name = "Green";
      color = ignition::math::Color::Green;
      break;
    case 3:
      color_name = "Magenta";
      color = ignition::math::Color::Magenta;
      break;
    case 4:
      color_name = "Red";
      color = ignition::math::Color::Red;
      break;
    case 5:
      color_name = "Yellow";
      color = ignition::math::Color::Yellow;
      break;
    case 6:
      color_name = "White";
      color = ignition::math::Color::White;
      break;
    }
  }
};

namespace gazebo{
  
  /// \brief A plugin to control a Filter sensor.
  class FilterPlugin : public ModelPlugin{

  private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Name of the model.
    string model_name;

    /// \brief Name of the model's parent.
    string parent_name;
    
    /// \brief Pointer to the world.
    physics::WorldPtr world;

    /// \brief An array of particles
    vector<Particle> particles;

    /// \brief A node use for Gazebo transport
    transport::NodePtr gzNode;

    // \brief Visual topic  publisher 
    transport::PublisherPtr visualPub;
    
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

    /// \brief Rng generator
    default_random_engine generator;

    /// \brief Rng distribution for particles color
    uniform_int_distribution<int> distribution_color;

    /// \brief Rng distribution for particles direction
    uniform_int_distribution<int> distribution_dir;

    volatile int P = 0; // index of current particle in simulation
  
  public:
    static const int NB_OBS = 3; // number of sensors (cf main.cpp)
    int N = 10; // number of particles used by default
    
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
      this->model_name = this->model->GetName();
      this->parent_name = "";

      // Store the world pointer for convenience.
      this->world = this->model->GetWorld();

      // Check that sdf elements exist, then read the values
      if (_sdf->HasElement("velocity"))
	MyRobot::MAX_SPEED = _sdf->Get<double>("velocity");
      if (_sdf->HasElement("nb_particles"))
	N = _sdf->Get<double>("nb_particles");

      // Random initialization of N particles
      random_device rd;
      generator = default_random_engine(rd());
      distribution_color = uniform_int_distribution<int>(0, 6);
      distribution_dir = uniform_int_distribution<int>(0, 3);
      uniform_real_distribution<float> distribution_width(-MyRobot::MAX_WIDTH + MyRobot::SIZE_ROBOT, MyRobot::MAX_WIDTH - MyRobot::SIZE_ROBOT);
      uniform_real_distribution<float> distribution_height(-MyRobot::MAX_HEIGHT + MyRobot::SIZE_ROBOT, MyRobot::MAX_HEIGHT - MyRobot::SIZE_ROBOT);
      uniform_real_distribution<float> distribution_angle(0, 6.28319);
      
      for(int i = 0; i < N; i++){
	float posX = distribution_width(generator);
	float posY = distribution_height(generator);
	float angle = distribution_angle(generator);
	int color_idx = distribution_color(generator);
	
	Particle p(posX, posY, angle, color_idx, N, NB_OBS);
	particles.push_back(p);
      }

      // Initialize transport
      this->gzNode = transport::NodePtr(new transport::Node());
      this->gzNode->Init();

      // Gazebo publisher for changing visual of robot
      this->visualPub = this->gzNode->Advertise<gazebo::msgs::Visual>("/gazebo/" + this->world->Name() + "/visual");

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
							    "/cmd_vel",
							    1,
							    boost::bind(&FilterPlugin::on_cmd, this, _1),
							    ros::VoidPtr(), &this->rosQueue);

      this->rosSub_cmd = this->rosNode->subscribe(so_cmd);

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so_cam =
	ros::SubscribeOptions::create<gazebo_particle_filter::Sensor>(
								      "/my_robot/camera",
								      100,
								      boost::bind(&FilterPlugin::on_camera, this, _1),
								      ros::VoidPtr(), &this->rosQueue_cam);
      
      this->rosSub_cam = this->rosNode->subscribe(so_cam);

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so_cam_filter =
	ros::SubscribeOptions::create<gazebo_particle_filter::Sensor>(
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
    void on_cmd(const geometry_msgs::TwistConstPtr &data)
    {
      for(int p = 0; p < N; p++)
	setPose(p);
    }

    /// \brief Handle an incoming message from ROS camera_depth
    /// \param[in] msg ROS camera_depth data
    void on_camera(const gazebo_particle_filter::SensorConstPtr &msg)
    {
      int i;
      // Check if all particles have non-zero observation data
      for(i = 0; i < N; i++)
	for(int o = 0; o < particles[i].obs.size(); o++)
	  if(particles[i].obs[o] == 0.0)
	    return;
      
      for(i = 0; i < N; i++)
	for(int o = 0; o < particles[i].obs.size(); o++)
	  particles[i].weight *= fctObservation(msg->data[o], particles[i].obs[o]);

      float sum = 0.0;
      for(i = 0; i < N; i++)
	sum += particles[i].weight;
      
      // Normalize particles weigth
      float max_weight = numeric_limits<float>::min();
      vector<float> weights;
      for(i = 0; i < N; i++){
	particles[i].weight /= sum;
	max_weight = max(max_weight, particles[i].weight);
	weights.push_back(particles[i].weight);
	cout << "Particle " << i << " (" << particles[i].color_name << ") : " << particles[i].weight << endl;
      }

      cout << endl;

      // Resampling
      if(max_weight >= 0.5){
	vector<Particle> new_particles;
	vector<float> cum_sum(weights.size());
	uniform_real_distribution<float> distribution_weight(0.0, 1.0);
	partial_sum(weights.begin(), weights.end(), cum_sum.begin());
	for(i = 0; i < N; i++){
	  float w = distribution_weight(generator);
	  int color_idx = distribution_color(generator);
	  vector<float>::iterator it = find_if(cum_sum.begin(), cum_sum.end(), [&w](float weight){
	      return w <= weight;
	    });
	  int idx = it - cum_sum.begin();
	  Particle p(particles[idx], color_idx, N);
	  new_particles.push_back(p);
	}

	particles = new_particles;
      }
    }

    /// \brief Handle an incoming message from ROS camera_depth particle
    /// \param[in] msg ROS camera_depth data
    void on_camera_filter(const gazebo_particle_filter::SensorConstPtr &msg)
    {
      particles[P].obs = msg->data;
    }

    /// \brief Set the new pose of a particle
    /// \param[in] p Particle
    /// \param[in] r Right target velocity
    /// \param[in] l Left target velocity
    void setPose(const int &p)
    {
      P = p; // set the current particle in simulation

      // Change color of model
      gazebo::msgs::Visual visualMsg;
      gazebo::msgs::Material *material = new gazebo::msgs::Material();
      gazebo::msgs::Color *color= new gazebo::msgs::Color();
      ignition::math::Color color_p = particles[P].color;
      visualMsg.set_name(this->model_name);
      visualMsg.set_parent_name(this->parent_name);
      color->set_r(color_p.R());
      color->set_g(color_p.G());
      color->set_b(color_p.B());
      material->set_allocated_ambient(color);
      visualMsg.set_allocated_material(material);
      this->visualPub->Publish(visualMsg);

      // Set pose of robot according to particle p pose
      ignition::math::Pose3d initPose(ignition::math::Vector3d(particles[p].posX, particles[p].posY, 0.), ignition::math::Quaterniond(0., 0., particles[p].angle));
      this->model->SetWorldPose(initPose);

      // Set random direction
      float r = 0.0, l = 0.0;
      
      switch(distribution_dir(generator)){
      case 0:
	// Forward
	r = l = MyRobot::MAX_SPEED;
	break;
      case 1:
	// Backward
	r = l = -MyRobot::MAX_SPEED;
	break;
      case 2:
	// Left
	r = MyRobot::MAX_SPEED;
	l = -MyRobot::MAX_SPEED;
	break;
      case 3:
	// Right
	r = -MyRobot::MAX_SPEED;
	l = MyRobot::MAX_SPEED;
	break;
      }
      
      this->model->GetJoint("my_robot_filter::left_wheel_hinge")->SetVelocity(0, l);
      this->model->GetJoint("my_robot_filter::right_wheel_hinge")->SetVelocity(0, r);

      // Move particle p in world 
      this->world->Step(1000); // nb of iterations

      // Get pose of robot after n iterations and set new pose of particle p
      auto model_pose = this->model->WorldPose();
      auto pose = model_pose.Pos();
      auto rot = model_pose.Rot();
      particles[p].posX = pose.X();
      particles[p].posY = pose.Y();
      particles[p].angle = rot.Yaw();
    }

    float fctObservation(float obs, float obs_p){
      float res = 0.0;
      
      if(obs == MyRobot::V_MIN)
	res = cdf(0.5, obs_p, MyRobot::STD_DEV);
      else if(obs == MyRobot::V_MAX)
	res = 1 - cdf(obs - 0.5, obs_p, MyRobot::STD_DEV);
      else 
	res = pdf(obs, obs_p, MyRobot::STD_DEV);

      return res;
    }

    // Cumulative distribution function of normal distribution
    float cdf(float x, float mean, float std_dev){
      return 0.5 * (1 + erf((x - mean) / (std_dev * sqrt(2))));
    }

    // Probability density function of normal distribution
    float pdf(float x, float mean, float std_dev){
      return 1 / (std_dev * sqrt(2 * M_PI)) * exp(-pow(x - mean, 2) / (2 * pow(std_dev, 2)));
    }

  private:
    /// \brief ROS helper function that processes cmd_vel messages
    void QueueThread()
    {
      static const double timeout = 0.01;
      while(this->rosNode->ok()){
	this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// \brief ROS helper function that processes camera messages
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
