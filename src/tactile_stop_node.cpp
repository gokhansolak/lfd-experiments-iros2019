// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


/**
  * This node listens for optoforce tactile data
  * if any finger has a signal higher than the threshold
  * then that finger is stopped.
  * The node takes the current control signal as input
  * and outputs the modified control signal.
  *
  * -- Gokhan
**/

#include <signal.h>
#include <boost/bind.hpp>
#include <ros/ros.h>

#include <allegro_hand_kdl/allegro_kdl_config.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

using namespace std;

vector<double> f_norm_vec_;
vector<bool> stop_vec_;

ros::Publisher torque_pub_;

bool finished_ = false;

// params
double threshold_ = 10;

bool getParams();
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void tactileForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, int index);
bool restartRequest(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
void sigintCallback(int sig);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tactile_stop");

    ros::NodeHandle nh;

    ROS_INFO("Tactile stop node is started.");

    // attempt to get ros parameters
    if(!getParams()) return -1;

    // fill initial forces with zeros
    f_norm_vec_.resize(allegro_hand_kdl::FINGER_COUNT, 0.0);
    stop_vec_.resize(allegro_hand_kdl::FINGER_COUNT, false);

    // create tactile data subscribers
    vector<ros::Subscriber> tactile_subs;
    for(int fi=0; fi<allegro_hand_kdl::FINGER_COUNT; fi++)
      tactile_subs.push_back(
        nh.subscribe<geometry_msgs::WrenchStamped>( "/optoforce_wrench_"+to_string(fi), 1, boost::bind(tactileForceCallback, _1, fi))
      );

    // create command signal subscriber
    ros::Subscriber cmd_sub =
      nh.subscribe<sensor_msgs::JointState>( "torque_in", 1, jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

    // create modified command publisher
    torque_pub_ =
      nh.advertise<sensor_msgs::JointState>("torque_out", 1);

    // restart server for reactivating fingers
    ros::ServiceServer serv = nh.advertiseService("restart_tactile_stop", restartRequest);

    // interrupt signal handler
    signal(SIGINT, sigintCallback);

    ros::spin();

    return 0;
}

bool getParams(){

  string param_name;

  // stop force threshold
  if(!ros::param::get("~threshold", threshold_)){
    ROS_WARN("Grasp node: Can't get threshold param.");
		return false;
  }
  ROS_DEBUG("Grasp node: threshold is %.3f.", threshold_);

  return true;

}

// Current command signal comes as a JointState message.
// We read it and stop fingers (substitute with zeros) if necessary.
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  // do nothing if finished
  if(finished_) return;

  // copy JointState msg
  sensor_msgs::JointState msg_new = *msg;

  // print stopping info,
  string str_stop = "Tactile stop:";

  bool is_updated = false;

  // loop over fingers
  int finger_len = allegro_hand_kdl::FINGER_LENGTH;
  for(int fi=0; fi<allegro_hand_kdl::FINGER_COUNT; fi++) {
    // touching something?
    if(stop_vec_[fi] || f_norm_vec_[fi] > threshold_){
      // zero joint torques for this finger
      for(int si=0; si<finger_len; si++)
        msg_new.effort[fi * finger_len + si] = 0.0;

      // remember stop state
      if(!stop_vec_[fi]){
        stop_vec_[fi] = true;
        is_updated = true;
      }
      // 0 means stopped
      str_stop += " 0";
    }else
      str_stop += " 1";
  }

  if(is_updated)
    cout << str_stop << ".\n";

  // publish the new msg
  torque_pub_.publish(msg_new);
}

// Tactile callback is called with different index for each finger
// it only updates the tactile force amplitude
void tactileForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, int index) {
  // calculate norm and assign to the vector
  f_norm_vec_[index] = pow(msg->wrench.force.x, 2);
  f_norm_vec_[index] += pow(msg->wrench.force.y, 2);
  f_norm_vec_[index] += pow(msg->wrench.force.z, 2);
  f_norm_vec_[index] = sqrt(f_norm_vec_[index]);
}


// Reactivates all fingers on request
bool restartRequest(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

  ROS_INFO("Tactile stop: restarted.");
  for(int fi=0; fi<allegro_hand_kdl::FINGER_COUNT; fi++)
    stop_vec_[fi] = false;

  return true;
}

void stopAll(){
  // send and empty torque message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);
  msg.effort.resize(16);

  for (int j=0; j < 16; j++){
    msg.name.push_back("joint_"+to_string(j+1));
  }

  torque_pub_.publish(msg);
}

// This procedure is called when the ros node is interrupted
void sigintCallback(int sig)
{

  // send an empty torque message
  stopAll();

  // set finished state
  finished_ = true;

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}
