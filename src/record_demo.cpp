// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#include <ros/ros.h>

#include <dmp_tools/trajectory_recorder.h>
#include <dmp_tools/joint_state_listener.h>
#include <lfd_experiments/frame_state_listener.h>

using namespace ros_dmp_tools;
using namespace lfd_experiments;
using namespace std;

// record parameters
double duration_ = 5.0;
int step_count_ = 200;
int steps_per_second_ = 20;

string traj_name_;

// state options
std::string state_type_;
int state_size_;

StateListener * state_listener_;

bool getParams();

int main(int argc, char **argv) {
  ros::init(argc, argv, "record_demo");

  ros::NodeHandle node_handle;
	ros::start();

  // get ros params duration and frequency
  getParams();

 	// countdown
  ros::Duration sleep_time(1.0);
	for (int ci = 3; ci > 0; ci--){
	  ROS_INFO( "Record demo: starting in %d",ci );
	  sleep_time.sleep();
	}

	ROS_INFO( "Record demo: started!" );

	ros_dmp_tools::TrajectoryRecorder trajectory_recorder(&node_handle, traj_name_);

	// TODO: choose a state listener type
  if(state_type_== "joint") {
  	state_listener_ = new JointStateListener(&node_handle);
  } else if(state_type_== "object"){
  	state_listener_ = new FrameStateListener(&node_handle);
  } else if(state_type_== "fingertip"){
      ROS_ERROR("Not implementation yet.");
      return -1;
  }

	state_listener_->startListening();
	trajectory_recorder.startRecording(state_size_, duration_, step_count_);

	ros::spin();

  delete state_listener_;
  return 0;
}

// get external parameters if possible
bool getParams(){

  string param_name;
  //*********
  if(!ros::param::get("~record_time", duration_) ){
    ROS_ERROR("Record demo: Can't find record_time param.");
    return -1;
  }
  ROS_INFO( "Record demo: duration: %f", duration_ );

  //*********
  if(!ros::param::get("~steps_per_second", steps_per_second_) ){
    ROS_ERROR("Record demo: Can't find steps_per_second_ param.");
    return -1;
  }
  ROS_INFO( "Record demo: steps per second: %d", steps_per_second_ );

  step_count_ = duration_ * steps_per_second_;

  //*********
  if(!ros::param::get("~state_type", state_type_) ){
    ROS_ERROR("Record demo: Can't find state_type param.");
    return -1;
  }
  ROS_INFO( "Record demo: state type: %s", state_type_.c_str() );

  //*********
  if(!ros::param::get("~state_size", state_size_) ){
    ROS_ERROR("Record demo: Can't find state_size param.");
    return -1;
  }
  ROS_INFO( "Record demo: state size: %d", state_size_ );


  //*********
  if(!ros::param::get("~trajectory_name", traj_name_) ){
    traj_name_ = "traj.txt";
  }
  ROS_INFO( "Record demo: state trajectory_name: %s", traj_name_.c_str());

  return true;

}
