// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

/*********************************************************************
* This is an example node, that uses the SpringServer to
* apply envelop force (for grasping) with an Allegro Hand robot.
*********************************************************************/
#include <signal.h>
#include <memory>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>

#include <allegro_hand_kdl/cartesian_force.h>
#include <kdl_control_tools/kdl_helper.h>

#include <spring_framework/spring_display.h>
#include <spring_framework/spring_server.h>


using namespace std;
using namespace allegro_hand_kdl;
using namespace spring_framework;

// spring resources
SpringServer *spring_server_;
shared_ptr<SpringNetwork> spring_network_;

// control parameters
double stiffness_ = 1.0; // get from rosparam
double rest_rate_ = 1.0; // ratio of the rest length to current distance

// flags
bool first_loop_ = true;

void springUpdateCallback(const SpringServer*, const vector<KDL::Frame>&);
void createSpringNetwork();
bool getParams();
void sigintCallback(int sig);

/**************************
* Envelop spring node is a ros node that applies enveloping force generated
* by the virtual spring framework in order to grasp the manipulated object,
* typically during demonstration.
* When started, it calculates the distance between the virtual object center
* and initializes the virtual spring rest lengths accordingly. So, the objects
* should already be in grasp when it starts.
* It uses the SpringServer to achive this.
**************************/
int main(int argc, char** argv){

  ros::init(argc, argv, "envelop_spring_node");

  shared_ptr<ros::NodeHandle> nh = make_shared<ros::NodeHandle>();

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // create spring network and server node
  // server takes the network by reference;
  // we can update the network, without informing the server.
  createSpringNetwork();
  spring_server_ = new spring_framework::SpringServer(FINGER_COUNT, spring_network_);
  // set callback that is called at each update of spring server
  function<void(const SpringServer*, const vector<KDL::Frame>&)> f = springUpdateCallback;
  spring_server_->setUpdateEnterHandler(f);
  // start server node
  spring_server_->start(nh);

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  ros::spin();

  // clean up
  delete spring_server_;

  ROS_INFO("Safely terminating Envelop Spring Node.");
}

// get external parameters if possible
bool getParams(){

  // stiffness
  if(!ros::param::get("~stiffness", stiffness_)){
    ROS_WARN("Envelop spring: Can't get stiffness param.");
  }
  ROS_DEBUG("Envelop spring: stiffness is %.3f.", stiffness_);

  // rest_rate
  if(!ros::param::get("~rest_rate", rest_rate_)){
    ROS_WARN("Envelop spring: Can't get rest rate param.");
  }
  ROS_DEBUG("Envelop spring: rest rate is %.3f.", rest_rate_);

  return true;

}

void springUpdateCallback(const SpringServer*, const vector<KDL::Frame>&){
  // if first loop, setup spring lengths
 if(first_loop_){

   // set rest lengths to current distances x rest_rate_
   spring_network_->resetRestLengths(rest_rate_);

   // except for the anti-collison springs
   spring_network_->resetRestLength(3, 1.0);
   spring_network_->resetRestLength(4, 1.0);

   first_loop_ = false;
 }
}

void createSpringNetwork(){

  spring_network_ = make_shared<SpringNetwork>();
  // a springs between the thumb and opposing fingers
  spring_network_->createSpring(1, 4);
  spring_network_->createSpring(2, 4);
  spring_network_->createSpring(3, 4);

  // anti-collision springs between adjacent fingers
  spring_network_->createSpring(1, 2);
  spring_network_->createSpring(2, 3);

  spring_network_->setStiffness(stiffness_);

}

// This procedure is called when the ros node is interrupted
void sigintCallback(int sig)
{
  // close the spring server safely
  spring_server_->stop();

  // spin once to handle communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}
