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

#include <memory>
#include <signal.h>

#include <lfd_experiments/TrajectoryRequest.h>

#include <allegro_hand_kdl/template_trajectory_control.h>

#include <spring_framework/spring_control.h>

#include <dmp_tools/dmp_loader.h>
#include <dmp_tools/line_tracer.h>
#include <dmp_tools/record_conversion.h>

#include <kdl_control_tools/progress_logger.h>
#include <kdl_control_tools/trajectory.h>
#include <kdl_control_tools/multi_robot_trajectory.h>
#include <kdl_control_tools/WrenchArray.h>

#include <kdl_conversions/kdl_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace allegro_hand_kdl;

typedef unique_ptr<ros_dmp_tools::LineTracer> LineTracerPtr;

unique_ptr<JointTrajectoryController> joint_traj_control_;

AllegroKdlConfig allegro_kdl_config_;
unique_ptr<Kinematics> kinematics_;

unique_ptr<ros_dmp_tools::DmpLoader> dmp_loader_;

ros::Publisher torque_pub_; // to control the robot
ros::Publisher logger_pub_;

// params
double safety_torque_;
string dmp_filename_;
double update_freq_ = 200;

// trajectory params
double duration_ = 10.0;
unique_ptr<KDL::Frame> custom_goal_;

// state
ros::Time tstart_;
vector<KDL::Frame> ff_vec_; // fingertip frames
KDL::Frame hand_pose_;
KDL::Frame vf_cur_; // virtual frame
vector<double> q_cur_; // joint states

shared_ptr<kdl_control_tools::JointTrajectory> joint_traj_;
kdl_control_tools::HandTrajectory hand_traj_;
kdl_control_tools::CartesianTrajectory obj_traj_;

// remember these are only for debugging / logging
KDL::Frame desired_state_;

bool init_dmp_ = false;
bool finished_ = true;

// debug tools
kdl_control_tools::ProgressLogger p_logger_;
vector<LineTracerPtr> tracers_;

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
void sigintCallback(int sig){

  // send an empty torque message
  stopMoving();

  // set finished state
  finished_ = true;

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishTorques(const vector<double>& torque_vec){
  // create a joint state message and publish torques
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);

  for (int j=0; j < 16; j++){

    // joint torque is sum of the user input and compensation torques.
    // If gravity compensation is not checked then torque_compensate will be all zeros
    double joint_torque = torque_vec[j];

    // emergency stop / inactive
    if(joint_torque > safety_torque_ || joint_torque < -safety_torque_){
      ROS_WARN("Task Trajectory Server: Too much torque! %.4f",joint_torque);
    }

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

/**
  * Calculates and prints the error between the desired final state
  * and the actual final state.
 **/
void printFinalError(const vector<double>& q_cur){
  // calculate joint state error
  JntArray q_des = joint_traj_->getPoint(joint_traj_->getLength()-1);
  cout << "joint errors: ";
  double err_total = 0;
  for(int ji=0; ji<16; ji++){
    double err = q_des(ji) - q_cur[ji];
    cout << err << ", ";
    err_total += abs(err);
  }
  cout << " total: " << err_total << endl;

    // calculate object pose error
  cout << "\nobject pose error: ";
  cout << (desired_state_.p - vf_cur_.p).Norm()*100 << ", ";
  cout << (desired_state_.M.UnitX() - vf_cur_.M.UnitX()).Norm() << " \n(cm)";
  cout << endl;
}


void publishLogs(double t_current){

  // create the message
  std_msgs::Float64MultiArray msg;

  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "traj_and_error";
  msg.layout.dim[0].size = 14;
  msg.layout.dim[0].stride = 14;

  int i;
  // current pose
  KDL::Vector rpy_cur;
  vf_cur_.M.GetRPY(rpy_cur[0], rpy_cur[1], rpy_cur[2]);
  for(i=0; i<3; i++)
    msg.data.push_back(vf_cur_.p(i));
  for(i=0; i<3; i++)
    msg.data.push_back(rpy_cur(i));

  // desired pose
  int current_step = obj_traj_.getIndex(t_current);
  KDL::Frame vf_des = obj_traj_.getPoint(current_step);
  KDL::Vector rpy_des;
  vf_des.M.GetRPY(rpy_des[0], rpy_des[1], rpy_des[2]);
  for(i=0; i<3; i++)
    msg.data.push_back(vf_des.p(i));
  for(i=0; i<3; i++)
    msg.data.push_back(rpy_des(i));

  // calculate object pose error
  KDL::Vector e_p = vf_des.p - vf_cur_.p;
  msg.data.push_back(e_p.Norm()*100);
  KDL::Vector e_r = vf_des.M.UnitX() - vf_cur_.M.UnitX();
  msg.data.push_back(e_r.Norm());

  logger_pub_.publish(msg);

  // trace the trajectories for debug
    // desired vf
  geometry_msgs::Point pos_cur;
  tf::pointKDLToMsg(vf_des.p, pos_cur);
  tracers_[0]->publishMarker(pos_cur);
    // finger traces
  for(int fi=0; fi<FINGER_COUNT; fi++){
    tf::pointKDLToMsg(hand_traj_[fi].getPoint(current_step).p, pos_cur);
    tracers_[fi+1]->publishMarker(pos_cur);
  }

}
/*********************************************************************
* Set a task trajectory to follow. It requires task state and its
* derivative, plus the time series. It will be converted to joint-space.
*********************************************************************/
void createJointTrajectory(const JntArray& q_init, const kdl_control_tools::HandTrajectory& hand_traj) {

  // allocate joint trajectory vectors
  int traj_len = hand_traj.getLength();

  // create empty
  joint_traj_ = make_shared<kdl_control_tools::JointTrajectory>();

  // copy size and times
  joint_traj_->setLength(traj_len);
  joint_traj_->setTimeArray(hand_traj.getTimeArray());

  // q_last starts from q_init
  JntArray q_last(q_init);

  // report progress
  cout << "Converting trajectory [";
  int ten_percents=0;

  // convert to a joint state trajectory
  // by applying IK at each step
  for(int ti=0; ti<traj_len; ti++){

    JntArray q_ik;
    JntArray qd_ik;

    // calculate joint pos by inverse kinematics
    int err;
    err = kinematics_->calcJointPos(q_last, hand_traj.getPoint(ti), q_ik);

    // TODO: ensure ik quality
    // TODO: velocity calc using IK

    // calculate joint velocity by numerical differentiation
    if(ti == 0)
      qd_ik = JntArray(q_ik.rows());
    else
      qd_ik.data = (q_ik.data-q_last.data)/hand_traj.getTime(ti);

    // assign to trajectory
    joint_traj_->setPoint(ti, q_ik);
    joint_traj_->setVelocity(ti, qd_ik);

    // remember the last q
    q_last = q_ik;

    // report progress
    if(double(ti)/traj_len > ten_percents*0.1){
      cout << "+";
      cout.flush();
      ten_percents++;
    }
  }

  cout << "] done.\n";

}
/*********************************************************************
* Set an object trajectory to follow. It requires object state and its
* derivative, plus the time series. Fingertip frames ff is also needed
* to calculate fingertip trajectories. q_init is the joint states
* vector to initiate IK solving. It will be converted to joint-space.
*********************************************************************/
void createFingerTrajectories(const JntArray& q_init, const vector<KDL::Frame>& ff) {
  // get the trajectory size and create fingertip trajectory vectors
  int traj_len = obj_traj_.getLength();

  hand_traj_.resize(FINGER_COUNT);
  hand_traj_.setLength(traj_len);

  // timestep loop
  for(int ti=0; ti<traj_len; ti++){

    // finger loop
    for(int fi=0; fi<FINGER_COUNT; fi++){
      // transform fingertip frame from object frame to palm frame
      hand_traj_[fi].setPoint(ti, obj_traj_.getPoint(ti) * ff[fi]);
      // linear velocity is same for all fingers
      hand_traj_[fi].setVelocity(ti, obj_traj_.getVelocity(ti)); // FIXME: works only for linear velocity

      // TODO: Acceleration
    }

    // set all time to the same value
    hand_traj_.setTime(ti, obj_traj_.getTime(ti));

  }

  ROS_INFO("Task Trajectory Server: converting object trajectory to joint-space.");
  createJointTrajectory(q_init, hand_traj_);
}

void createObjectTrajectory( const KDL::Frame& vf){

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

  // report trajectory ends
  double roll, pitch, yaw;
    vf.M.GetRPY(roll, pitch, yaw);
    ROS_INFO("Task Trajectory Server: init state pos: %.4f, %.4f, %.4f. rpy: %.3f, %.3f, %.3f",
        vf.p.x(), vf.p.y(), vf.p.z(),
        roll, pitch, yaw);
    desired_state_.M.GetRPY(roll, pitch, yaw);
    ROS_INFO("Task Trajectory Server: goal state pos: %.4f, %.4f, %.4f. rpy: %.3f, %.3f, %.3f",
        desired_state_.p.x(), desired_state_.p.y(), desired_state_.p.z(),
        roll, pitch, yaw);

  // for debugging
  dmp_loader_->saveLastTrajectory();

}

void initializeControl(){
  // fingertip frames w.r.t. the object frame vf
  vector<KDL::Frame> ff_on_vf_vec;
  for(int fi=0; fi<FINGER_COUNT; fi++)
    ff_on_vf_vec.push_back(vf_cur_.Inverse() * ff_vec_[fi]);

  // create and set the trajectory
  createObjectTrajectory(vf_cur_);
  createFingerTrajectories(kdl_control_tools::vectorStdToKdl(q_cur_), ff_on_vf_vec);

  joint_traj_control_->setTrajectory(joint_traj_);

  ROS_INFO("Task Trajectory Server: created %s trajectory.", dmp_filename_.c_str());

  // start the timer
  tstart_ = ros::Time::now();
}

// read the current base frame pose (hand root wrt ur5 base)
void baseposeCallback(const geometry_msgs::Pose::ConstPtr &msg){
  // update the hand pose
  tf::poseMsgToKDL(*msg, hand_pose_);
}

// update the fingertip states
void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {
  // avoid working after finishing
  if(finished_) return;

  ff_vec_.resize(FINGER_COUNT);

  // convert finger poses to kdl frames
  vector<KDL::Frame> ff_vec(FINGER_COUNT);
  for(int fi=0; fi < FINGER_COUNT; fi++){
    tf::poseMsgToKDL(msg->poses[fi], ff_vec_[fi]);
  }
  // update virtual frame
  vf_cur_ = spring_framework::SpringController::computeVirtualFrame(ff_vec_);
}

/**
  * read the latest joint states, that is used by the controller to get
  * the cartesian finger positions
 **/
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  // avoid working after finishing
  if(finished_) return;
  // read the joint states
  q_cur_ = msg->position;
}

/**
  * Main update loop, called by a timer. It uses the data acquired in other
  * callbacks to run the controller.
 **/
void updateCallback(const ros::TimerEvent&){
  // avoid working after finishing
  if(finished_ || ff_vec_.size() == 0 || q_cur_.size() == 0) return;

  // *** initialize a dmp if requested & read fingertip frames already
  if(init_dmp_){

    initializeControl();

    init_dmp_ = false;
    return;
  }

  // control

  // Calc time since the beginning of control
  ros::Time tnow_ = ros::Time::now();
  double dt_start = (tnow_ - tstart_).sec + 1e-9 * (tnow_ - tstart_).nsec;

  publishLogs(dt_start);
  // publishDesiredMotion(dt_start);

  // Termination time?
  if (dt_start > duration_){
    ROS_INFO("Task Trajectory Server: finished");
    printFinalError(q_cur_);

    finished_ = true;
    stopMoving();
    return;
  }

  // calculate torques to follow trajectory
  vector<double> torques;
  joint_traj_control_->computeTorques(dt_start, q_cur_, torques);

  publishTorques(torques);

}

/**
  * listen for trajectory requests, initiate dmp execution
 **/
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
    // convert to hand orientation
    custom_goal_->p = hand_pose_.M * custom_goal_->p;
  }else
    custom_goal_.reset();

  ROS_INFO("Task Trajectory Server: dmp file to execute: %s.", dmp_filename_.c_str());

  // set the flags to activate controller and init dmp
  finished_ = false;
  init_dmp_ = true;
  // forget the possible remaining data from previous run
  ff_vec_.clear();
  q_cur_.clear();

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

bool readJointControlGains(){

  vector<double> k_pos_vec(FINGER_LENGTH*FINGER_COUNT);
  vector<double> k_vel_vec(FINGER_LENGTH*FINGER_COUNT);
  vector<double> k_int_vec(FINGER_LENGTH*FINGER_COUNT);

  string param_name = "/allegroHand_right_0/gains/trajectory";
  if(!ros::param::has(param_name) ){
    ROS_ERROR("Task Trajectory Server: Can't find gains/trajectory param.");
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

  joint_traj_control_->control->setGains(k_pos_vec, k_vel_vec, k_int_vec);

  return true;
}

// get external parameters if possible
bool getParams(){
  // safety_torque
  if(!ros::param::get("/allegro_kdl/safety_torque", safety_torque_)){
    ROS_ERROR("Task Trajectory Server: Can't get safety_torque param.");
    return false;
  }
  ROS_DEBUG("Task Trajectory Server: safety_torque is %.3f.", safety_torque_);

  // dmp filename to execute initially
  if(!ros::param::get("~dmp", dmp_filename_)){
    ROS_WARN("Task Trajectory Server: Can't get dmp param.");
  }
  if(dmp_filename_.length() > 0)
    ROS_INFO("Task Trajectory Server: dmp file to execute: %s.", dmp_filename_.c_str());

  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "joint_trajectory_server");
  ROS_INFO("Task Trajectory Server node");

  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner(2);

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // create kinematics solver
  allegro_kdl_config_.parseKdl(nh);
  kinematics_ = unique_ptr<Kinematics>( new Kinematics(allegro_kdl_config_));

  // finger activation binary number
  int active_fingers = 91111;
  // create controller object
  joint_traj_control_ = unique_ptr<JointTrajectoryController>( new JointTrajectoryController());
  // get PID gains
  if(!readJointControlGains()) return -1;
  // deactivate unused fingers
  joint_traj_control_->control->setActiveFingers(active_fingers);

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
      ROS_INFO( "Task Trajectory Server: starting in %d",ci );
      sleep_time.sleep();
    }
  }

  ROS_INFO( "Task Trajectory Server: started!");

  // create ros communication nodes
  torque_pub_ =
    nh.advertise<sensor_msgs::JointState>("traj_torque", 1);
  ros::Subscriber sub_js =
    nh.subscribe<sensor_msgs::JointState>( "joint_states", 1, jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());
  ros::Subscriber sub_pose =
    nh.subscribe<geometry_msgs::PoseArray>("pose_array", 1, poseArrayCallback, ros::TransportHints().tcpNoDelay().reliable());
  ros::Subscriber sub_base =
    nh.subscribe<geometry_msgs::Pose>("base_pose", 1, baseposeCallback);

  ros::ServiceServer service =
    nh.advertiseService("desired_task_traj", processTrajectoryRequest);

  // timer callback for the main loop
  ros::Timer timer = nh.createTimer(ros::Duration(1/update_freq_), updateCallback);

  // DEBUG:
  logger_pub_ =
    nh.advertise<std_msgs::Float64MultiArray>("task_traj_log", 3);

  // DEBUG:
  // virtual frame tracer
  tracers_.resize(FINGER_COUNT+1);
  tracers_[0].reset(new ros_dmp_tools::LineTracer("vf_des_trace"));
  tracers_[0]->setDuration(10.0);
  tracers_[0]->setColor(0.8, 0.6, 0.4, 0.6);
  tracers_[0]->setWidth(0.001);
  tracers_[0]->start(nh);
  // finger tracers
  for(int fi=0; fi<FINGER_COUNT; fi++){
    tracers_[fi+1].reset(new ros_dmp_tools::LineTracer("f"+to_string(fi)+"_des_trace"));
    tracers_[fi+1]->setDuration(10.0);
    tracers_[fi+1]->setColor(0.4, 0.6, 0.8, 0.6);
    tracers_[fi+1]->setWidth(0.0005);
    tracers_[fi+1]->start(nh);
  }

  ros::spin(spinner);

  return 0;

}
