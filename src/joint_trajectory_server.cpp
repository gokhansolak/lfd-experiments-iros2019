// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

/**
* This is a server node for JointTrajectoryController.
* It listens to and satisfies TrajectoryRequests.
* If a service request is issued, it loads the requested
* DMP and executes it according to desired parameters.
* It can also be given a ROS parameter to execute a
* trajectory on start.
**/

#include <signal.h>

#include <lfd_experiments/TrajectoryRequest.h>

#include <allegro_hand_kdl/template_trajectory_control.h>

#include <dmp_tools/dmp_loader.h>
#include <dmp_tools/trajectory_conversion.h>

#include <kdl_control_tools/trajectory.h>
#include <kdl_control_tools/progress_logger.h>
#include <kdl_control_tools/kdl_helper.h>

#include <sensor_msgs/JointState.h>

using namespace std;
using namespace allegro_hand_kdl;

JointTrajectoryController* traj_control_;

ros_dmp_tools::DmpLoader* dmp_loader_;

ros::Publisher torque_pub_; // to control the robot

// params
double safety_torque_;
string dmp_filename_;

ros::Time tstart_;

double duration_ = 10.0;

bool init_dmp_ = false;
bool finished_ = true;

kdl_control_tools::ProgressLogger p_logger_;

void stopMoving(){
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
  // save the error trajectory
  string filename =
      "_"+to_string((int)traj_control_->control->getPositionGain()[0])+"_"+
      to_string((int)traj_control_->control->getVelocityGain()[0])+".csv";

  p_logger_.writeVarToFile("dt", filename);

  // send an empty torque message
  stopMoving();

  // set finished state
  finished_ = true;

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishTorque(const vector<double>& torque_vec){
  // create a joint state message and publish torques
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);

  for (int j=0; j < 16; j++){

    // joint torque is sum of the user input and compensation torques.
    // If gravity compensation is not checked then torque_compensate will be all zeros
    double joint_torque = torque_vec[j];

    // shave torque instead of emergency stop
    joint_torque = max(joint_torque, -safety_torque_);
    joint_torque = min(joint_torque, safety_torque_);

    // add torques to message as effort
    msg.effort.push_back(joint_torque);
    // names of joints
    msg.name.push_back("joint_"+to_string(j+1));
  }

  torque_pub_.publish(msg);
}

void createJointTrajectory(vector<double> q_now){

  // determine the step count to have a desired step size
  int step_count = (int)(duration_ / 0.0015);

  // load dmp file
  dmp_loader_ = new ros_dmp_tools::DmpLoader(dmp_filename_);

  // get dmp trajectory with initial joint state
  kdl_control_tools::JointTrajectory traj_kdl;

  dmp_loader_->setInitialState(q_now);
  dmp_loader_->createTrajectory(step_count, duration_, traj_kdl);

  // for debugging
  dmp_loader_->saveLastTrajectory();

  // set the controller trajectory
  traj_control_->setTrajectory(
    std::make_shared<kdl_control_tools::JointTrajectory>(traj_kdl));
}


void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  // avoid working after finishing
  if(finished_) return;

  // *** initialize a dmp if requested
  if(init_dmp_){

    createJointTrajectory(msg->position);

    ROS_INFO("Joint Trajectory Server: created %s trajectory", dmp_filename_.c_str());

    // start the timer
  	tstart_ = ros::Time::now();

    init_dmp_ = false;
    return;
  }

  // *** during control

  // Calc time since last control
  ros::Time tnow_ = ros::Time::now();
  double dt_start = (tnow_ - tstart_).sec + 1e-9 * (tnow_ - tstart_).nsec;

  // Termination time?
  if (dt_start > duration_){
    ROS_INFO("Joint Trajectory Server: finished");
    finished_ = true;
    stopMoving();
    return;
  }

  // copy state
  vector<double> q = msg->position;

  // calculate torques to follow trajectory
  vector<double> torques;
  traj_control_->computeTorques(dt_start, q, torques);

  publishTorque(torques);

}

// listen for trajectory requests, initiate dmp execution
bool processTrajectoryRequest(
      lfd_experiments::TrajectoryRequest::Request& req,
      lfd_experiments::TrajectoryRequest::Response& res){

  // read the request, update the params
  dmp_filename_ = req.dmp;
    // update duration only if positive
  if(req.duration > 0) duration_ = req.duration;

  // TODO: custom goal request

  // set the flags to activate controller and init dmp
  finished_ = false;
  init_dmp_ = true;

  // wait until it finishes
  while(ros::ok()){
    ros::spinOnce();

    // TODO: time out fail
    res.success = true;
    // return success if finished
    if(finished_)
      return true;
  }

}

bool getControlGains(ros::NodeHandle& nh, vector<double>& k_pos_vec, vector<double>& k_vel_vec, vector<double>& k_int_vec){

  k_pos_vec.resize(FINGER_LENGTH*FINGER_COUNT);
  k_vel_vec.resize(FINGER_LENGTH*FINGER_COUNT);
  k_int_vec.resize(FINGER_LENGTH*FINGER_COUNT);

  string param_name;
  if(!nh.searchParam("/allegroHand_right_0/gains/trajectory", param_name) ){
    ROS_ERROR("Joint Trajectory Server: Can't find trajectory gains param.");
    return false;
  }

  for(int fi=0; fi < FINGER_COUNT; fi++)
    for(int si=0; si < FINGER_LENGTH; si++){
      ros::param::get(param_name+"/p/j"+to_string(fi)+to_string(si),
        k_pos_vec[fi*FINGER_LENGTH+si]);
      ros::param::get(param_name+"/d/j"+to_string(fi)+to_string(si),
        k_vel_vec[fi*FINGER_LENGTH+si]);
      ros::param::get(param_name+"/i/j"+to_string(fi)+to_string(si),
        k_int_vec[fi*FINGER_LENGTH+si]);
    }

  return true;
}


// get external parameters if possible
bool getParams(){
  // safety_torque
  if(!ros::param::get("/allegro_kdl/safety_torque", safety_torque_)){
    ROS_WARN("Joint Trajectory Server: Can't get safety_torque param.");
    return false;
  }
  ROS_DEBUG("Joint Trajectory Server: safety_torque is %.3f.", safety_torque_);

  // dmp filename to execute initially
  if(!ros::param::get("~dmp", dmp_filename_)){
    ROS_WARN("Joint Trajectory Server: Can't get dmp param.");
  }
  ROS_INFO("Joint Trajectory Server: dmp file to execute: %s.", dmp_filename_.c_str());

  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "joint_trajectory_server");
  ROS_INFO("Joint Trajectory server node");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // create controller object
  traj_control_ = new JointTrajectoryController();
  traj_control_->control->connectLogger(&p_logger_);

  // get the pd gain params if exists
  vector<double> k_pos_vec;
  vector<double> k_vel_vec;
  vector<double> k_int_vec;

  if(getControlGains(nh, k_pos_vec, k_vel_vec, k_int_vec)){
    traj_control_->control->setGains(k_pos_vec, k_vel_vec, k_int_vec);
  }else{
    traj_control_->control->setGains(0.4, 0.05, 0.4);
  }

  // decrease the desired velocity
  traj_control_->control->setDamping(0.0);

  // deactivate some fingers
  traj_control_->control->setActiveFingers(91111);

  // run the initial dmp file if requested
  if(dmp_filename_.length() > 0){
    finished_ = false;
    init_dmp_ = true;
  }

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // countdown only for an initial trajectory
  if(init_dmp_){
    ros::Duration sleep_time(1.0);

    for (int ci = 3; ci > 0; ci--){
      ROS_INFO( "Joint Trajectory Server: starting in %d",ci );
      sleep_time.sleep();
    }
  }

  ROS_INFO( "Joint Trajectory Server: started!");

  // create ros communication nodes
  torque_pub_ =
    nh.advertise<sensor_msgs::JointState>("traj_torque", 3);
  ros::Subscriber sub =
    nh.subscribe<sensor_msgs::JointState>( "joint_states", 3, jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

  ros::ServiceServer service =
    nh.advertiseService("desired_traj", processTrajectoryRequest);

  ros::spin();

  delete traj_control_;
  delete dmp_loader_;

  return 0;

}
