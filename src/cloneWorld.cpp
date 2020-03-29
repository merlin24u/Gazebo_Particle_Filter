#include "ros/ros.h"
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

void OnWorldModify(ConstWorldModifyPtr &_msg)
{
  if (_msg->has_cloned() && _msg->cloned() && _msg->has_cloned_uri()){
    std::cout << "World cloned. You can connect a client by typing\n"
              << "\tGAZEBO_MASTER_URI=" << _msg->cloned_uri()
              << " gzclient" << std::endl;
  }
  
  ROS_INFO("%s", "Ok cloné");
}

int main(int _argc, char **_argv){
  // Create a node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  
  // Publisher to the server control
  gazebo::transport::PublisherPtr serverControlPub =
    node->Advertise<gazebo::msgs::ServerControl>("/gazebo/server/control");

  
  // Subscriber to receive world updates (e.g.: a notification after a cloning)
  gazebo::transport::SubscriberPtr worldModSub =
    node->Subscribe("/gazebo/world/modify", &OnWorldModify);
  
  // Clone the server programmatically
  gazebo::msgs::ServerControl msg;
  msg.set_save_world_name(""); // default
  msg.set_clone(true);
  msg.set_new_port(11346);
  serverControlPub->Publish(msg);

  // Wait for the simulation clone
  gazebo::common::Time::MSleep(200);

  ROS_INFO("%s", "Fini");
}
