// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#include <signal.h>

#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>

#include <lfd_experiments/SpringUpdate.h>
#include <lfd_experiments/TrajectoryRequest.h>

#include <dmp_tools/dmp_loader.h>

#include <spring_framework/spring_control.h>
#include <spring_framework/spring_server.h>

#include <kdl_control_tools/kdl_helper.h>

using namespace std;
using namespace spring_framework;

// node
shared_ptr<ros::NodeHandle> nh_;

// dmp
unique_ptr<ros_dmp_tools::DmpLoader> dmp_loader_;
 // trajectory data
 kdl_control_tools::CartesianTrajectory obj_traj_;

// spring framework
shared_ptr<SpringNetwork> spring_network_;
SpringServer *spring_server_;

// control parameters (from ros params)
double k_pos_ = 20.0;
double k_rot_ = 10.0;
vector<double> stiffness_;
vector<double> rest_rates_;
double anti_collision_gain_ = 2.0;

KDL::Frame hand_pose_;
string dmp_filename_;

// communication
ros::Publisher marker_pub_; // to visualize the object pose

// timing
ros::Time t_start_;

// trajectory params
double duration_ = 10.0;
unique_ptr<KDL::Frame> custom_goal_;
KDL::Frame desired_state_;

bool finished_ = true;
bool init_dmp_ = false;
bool init_grasp_ = false;

// debug
kdl_control_tools::ProgressLogger p_logger_;


// This procedure is called when the ros node is interrupted
void sigintCallback(int sig)
{

  // close the spring server safely
  spring_server_->stop();

  // TODO: save the debug trajectory

  // set finished state
  finished_ = true;

  // spin once to handle communication
  ros::spinOnce();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishMarker(const KDL::Frame& vf, double reference=0){

  visualization_msgs::Marker marker;
  marker.header.frame_id = "hand_root";
  marker.header.stamp = ros::Time();
  marker.ns = "virtual_frame_pub";

  marker.id = reference;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.016;
  marker.scale.y = 0.016;
  marker.scale.z = 0.016;
  marker.color.r = 0.8;
  marker.color.g = 0.4 + reference*0.4;
  marker.color.b = 0.8 - reference*0.4;
  marker.color.a = 0.65;

  // position from arg
  tf::poseKDLToMsg(vf, marker.pose);

  // short duration
  marker.lifetime = ros::Duration(0.05);

  marker.frame_locked = true;

  marker_pub_.publish( marker );

}

void createTrajectory(KDL::Frame& vf){

  // convert KDL frame into RecordState format
  vector<double> rs_current;
  ros_dmp_tools::toRecordState(vf, rs_current);

  // determine the step count to have a desired step size
  int step_count = (int)(duration_ / 0.0015);

  // load dmp file
  dmp_loader_ =
    unique_ptr<ros_dmp_tools::DmpLoader>( new ros_dmp_tools::DmpLoader(dmp_filename_));

  // apply custom goal if exists
  if(custom_goal_){
    // because goal is relative to current frame combine it
    desired_state_.p = vf.p + custom_goal_->p;
    desired_state_.M = vf.M * custom_goal_->M;
    vector<double> rs_goal;
    ros_dmp_tools::toRecordState(desired_state_, rs_goal);
    dmp_loader_->setGoalState(rs_goal);
  }else{ // otherwise get the teached goal state
      // transformation info
    vector<double> rs_goal = dmp_loader_->getGoalState();
    ros_dmp_tools::toKDL(rs_goal, desired_state_);
  }

  // get dmp trajectory for this initial joint state
  dmp_loader_->setInitialState(rs_current);
  dmp_loader_->createTrajectory(step_count, duration_, obj_traj_);

  // for debugging
  dmp_loader_->saveLastTrajectory();

  ROS_INFO("Spring Trajectory Server: created dmp trajectory");
}

// return the reference frame and velocity for current time step
void readTrajectory(const double t_current, KDL::Frame& vf, KDL::Twist& vel){
  // Get values for the current trajectory step
  int current_step = obj_traj_.getIndex(t_current);

  vf = obj_traj_.getPoint(current_step);
  vel = obj_traj_.getVelocity(current_step);
}

// called at the beginning of each spring server update.
// sets the spring parameters and creates the trajectory at the beginning.
// updates the desired pose and velocity of object for spring server.
// publishes the object pose marker (TODO: move this to server?)
void springUpdateCallback(const SpringServer* server, const vector<KDL::Frame>& frames) {

  // get current virtual frame
  KDL::Frame vf = frames[0];

  // **************** INITIALIZE
  // grasp initiation upon request
  if(init_grasp_){
    // configure springs to grasp
    // set rest lengths to current distances x rest_rates_
    for(int fi=0; fi<4; fi++)
      spring_network_->resetRestLength(fi, rest_rates_[fi]);

    // update stiffness of contact springs
    for(int fi=0; fi<4; fi++)
      spring_network_->setStiffness(fi, stiffness_[fi]);

    // anti-collison springs are always at full rest
    // stiffness are not updated
    spring_network_->resetRestLength(4, 1.0);
    spring_network_->resetRestLength(5, 1.0);

    init_grasp_ = false;
  }
  // The trajectory is created if requested
  if(init_dmp_){
    // create the trajectory with current virtual frame as initial state
    createTrajectory(vf);

    // configure impedance control parameters
    spring_server_->setPositionGain(k_pos_);
    spring_server_->setRotationGain(k_rot_);

    // disable velocity control, because of strange behaviour:
    spring_server_->setVelocityGain(0.0);

    // display pose
    publishMarker(vf);

    // start the timer
  	t_start_ = ros::Time::now();

    init_dmp_ = false;
    return;
  }

  // avoid working after finishing
  if(finished_) return;

  // **************** CONTROL
  // time since last control
  ros::Time t_now = ros::Time::now();
  double dt_start = (t_now - t_start_).sec + 1e-9 * (t_now - t_start_).nsec;

  // Termination time?
  if (dt_start > duration_){
    ROS_DEBUG("Spring Trajectory Server: trajectory is completed");
    finished_ = true;
    // stop the movement
    spring_server_->setObjectControl(false);
    return;
  }

  // get the reference trajectory info from trajectory
  KDL::Frame vf_ref;
  KDL::Twist vel_ref;
  readTrajectory(dt_start, vf_ref, vel_ref);

  // update reference pose and velocity for spring server
  spring_server_->setDesiredPose(vf_ref);
  spring_server_->setDesiredVelocity(vel_ref.vel);

  // display current and desired poses
  publishMarker(vf);
  publishMarker(vf_ref, 1.0);

}


void createSpringNetwork(){

  spring_network_ = make_shared<SpringNetwork>();
  // a springs between the thumb and opposing fingers
  spring_network_->createSpring(1, 0);
  spring_network_->createSpring(2, 0);
  spring_network_->createSpring(3, 0);
  spring_network_->createSpring(4, 0);

  // anti-collision springs between adjacent fingers
  spring_network_->createSpring(1, 2);
  spring_network_->createSpring(2, 3);

  // initially there can be single stiffness value
  // finger-object spring stiffness may be updated
  spring_network_->setStiffness(stiffness_[0]);
  // anti-collision springs are more stiff by default
  // but they will always keep their initial value
  spring_network_->setStiffness(4, stiffness_[0]*anti_collision_gain_);
  spring_network_->setStiffness(5, stiffness_[0]*anti_collision_gain_);

}

// read the current base frame pose (hand root wrt ur5 base)
void baseposeCallback(const geometry_msgs::Pose::ConstPtr &msg){
  // update the hand pose
  tf::poseMsgToKDL(*msg, hand_pose_);
}

// listen for spring updates, initiate spring activation
bool processSpringUpdate(
      lfd_experiments::SpringUpdate::Request& req,
      lfd_experiments::SpringUpdate::Response& res){

  // re-init the grasp since stiffness and rest rate maybe changed
  // finish only if not active and for all fingers (index = -1)
  // TODO: activate finger springs independently
  init_grasp_ = req.active || req.index >= 0;


  // update all or a single finger
  if(req.index < 0){
    // ignore negative valued requests
    if(req.rest_length >= 0){
      rest_rates_.clear();
      rest_rates_.resize(4, req.rest_length);
    }
    // ignore negative valued requests
    if(req.stiffness >= 0){
      stiffness_.clear();
      stiffness_.resize(4, req.stiffness);
    }

  }else{
    // ignore negative valued requests
    if(req.rest_length >= 0)
      rest_rates_[req.index] = req.rest_length;
    // ignore negative valued requests
    if(req.stiffness >= 0)
      stiffness_[req.index] = req.stiffness;
  }

  // start stop / server
  if(init_grasp_) spring_server_->start(nh_);
  else spring_server_->stop();

  return true;

}

// listen for trajectory requests, initiate dmp execution
bool processTrajectoryRequest(
      lfd_experiments::TrajectoryRequest::Request& req,
      lfd_experiments::TrajectoryRequest::Response& res){

  // read the request, update the params
  dmp_filename_ = req.dmp;

  // update duration only if positive
  if(req.duration > 0) duration_ = req.duration;

  if(req.custom_goal){
    // assign goal from request
    custom_goal_.reset(new KDL::Frame());
    tf::poseMsgToKDL(req.cartesian_goal, *custom_goal_);
    // rotate to hand frame
    custom_goal_->p = hand_pose_.M * custom_goal_->p; // TODO: test with rotation change
  }else
    custom_goal_.reset();

  ROS_INFO("Spring Trajectory Server: dmp file to execute: %s.", dmp_filename_.c_str());

  // set the flags to activate controller and init dmp
  finished_ = false;
  init_dmp_ = true;

  // ensure grasping is active
  init_grasp_ = true;
  spring_server_->start(nh_);

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

// get external parameters if possible
bool getParams(){

  string param_name;

  // get the play duration
  ros::param::get("~play_time", duration_);
  ROS_INFO( "Spring Trajectory Server: duration: %.2f", duration_ );

  // get control gains
  ros::param::get("~k_pos", k_pos_);
  ROS_DEBUG( "Spring Trajectory Server: k_pos: %.2f", k_pos_ );
  ros::param::get("~k_rot", k_rot_);
  ROS_DEBUG( "Spring Trajectory Server: k_rot: %.2f", k_rot_ );
  // stiffness
  double k;
  if(ros::param::get("~stiffness", k)){
    ROS_DEBUG("Spring Trajectory Server: stiffness is %.3f.", k);
    // assign it to all fingers
    stiffness_.clear();
    stiffness_.resize(4, k);
  }else{
    ROS_WARN("Spring Trajectory Server: Can't get stiffness param.");
    //assign 1.0 by default
    stiffness_.clear();
    stiffness_.resize(4, 1.0);
  }
  // rest_rate
  double rr;
  if(ros::param::get("~rest_rate", rr)){
    ROS_DEBUG("Spring Trajectory Server: rest rate is %.3f.", rr);
    // assign it to all fingers
    rest_rates_.clear();
    rest_rates_.resize(4, rr);
  }else{
    ROS_WARN("Spring Trajectory Server: Can't get rest rate param.");
    //assign 1.0 by default
    rest_rates_.clear();
    rest_rates_.resize(4, 1.0);
  }
  // anti_collision_gain
  if(!ros::param::get("~anti_collision_gain", anti_collision_gain_)){
    ROS_WARN("Spring Trajectory Server: Can't get anti_collision_gain param.");
  }
  ROS_DEBUG("Spring Trajectory Server: anti_collision_gain is %.3f.", anti_collision_gain_);

  // dmp filename to execute initially
  if(!ros::param::get("~dmp", dmp_filename_)){
    ROS_WARN("Spring Trajectory Server: Can't get dmp param.");
  }
  ROS_INFO("Spring Trajectory Server: dmp file to execute: %s.", dmp_filename_.c_str());

  return true;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "spring_trajectory_server");
  ROS_INFO("Spring Trajectory Server started");

  nh_ = make_shared<ros::NodeHandle>();

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // TODO: get initial desired trajectory as param

  // create spring network and server node
  // server takes the network by reference;
  // we can update the network, without updating the server.
  createSpringNetwork();
  spring_server_ = new spring_framework::SpringServer(4, spring_network_);
  // set callback that is called at each update of spring server
  function<void(const SpringServer*, const vector<KDL::Frame>&)> f = springUpdateCallback;
  spring_server_->setUpdateEnterHandler(f);

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // countdown when there's an initial trajectory
  if(dmp_filename_.length() > 0){

    ros::Duration sleep_time(1.0);
    for (int ci = 3; ci > 0; ci--){
      ROS_INFO( "Spring Trajectory Server: starting in %d",ci );
      sleep_time.sleep();
    }

    // run the initial dmp file when requested
    finished_ = false;
    init_dmp_ = true;
    init_grasp_= true;
    spring_server_->start(nh_);
  }

  ROS_INFO( "Spring Trajectory Server: started!" );

  // visualization_msgs publisher for markers
  marker_pub_ =
    nh_->advertise<visualization_msgs::Marker>("virtual_frame_pub", 1);
  // needed to transform the goal pose
  ros::Subscriber sub_base =
    nh_->subscribe<geometry_msgs::Pose>("base_pose", 1, baseposeCallback);
  // services to get trajectory and spring grasp requests
  ros::ServiceServer srv_traj =
    nh_->advertiseService("desired_spring_traj", processTrajectoryRequest);
  ros::ServiceServer srv_spring =
    nh_->advertiseService("spring_update", processSpringUpdate);

  // wait for joint state messages
  ros::spin();

  delete spring_server_;

  return 0;

}
