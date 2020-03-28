#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

int main(int _argc, char **_argv){
  // Create a node for communication.
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  
  // Publisher to the server control.
  gazebo::transport::PublisherPtr serverControlPub =
    node->Advertise<gazebo::msgs::ServerControl>("/gazebo/server/control");
  
  // Clone the server programmatically.
  gazebo::msgs::ServerControl msg;
  msg.set_save_world_name(""); // default
  msg.set_clone(true);
  msg.set_new_port(11346);
  serverControlPub->Publish(msg);

  // Wait for the simulation clone
  gazebo::common::Time::MSleep(200);
}
