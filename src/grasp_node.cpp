// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

/* The program to run grasping experiments.
 * Takes user inputs for the experiment parameters.
 *
 * It uses blocking service calls to other robot specific nodes
 * to achieve each of these phases.
 *
 * --Gokhan Solak
*/

#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>

#include <cartesian_move.h>
#include <lfd_experiments/TrajectoryRequest.h>
#include <lfd_experiments/SpringUpdate.h>

#include <allegro_hand_kdl/PoseRequest.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

using namespace std;

string manipulator_name_;
string end_link_name_;

double velocity_scaler_;

unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

// comm
ros::ServiceClient hand_pose_client_;
ros::ServiceClient joint_traj_client_;
ros::ServiceClient task_traj_client_;
ros::ServiceClient hand_spring_client_;
ros::ServiceClient stiffness_adjuster_client_;

ros::Publisher label_pub_;

// action params
string action_name_;
geometry_msgs::Pose palm_pose_;
string pregrasp_name_;
string dmp_name_;
string dmp_path_;
string manipulate_type_;
// generalization
double action_time_;
unique_ptr<geometry_msgs::Pose> custom_goal_;

double z_approach_ = 0.08; // m
double z_pick_ = 0.04; // m

// spring params
double stiffness_;
double rest_length_;

geometry_msgs::Vector3 zero_point_;


bool getParams();
bool readGraspParams(int gi);
bool readManipulationParams(int gi);
bool printLibrary(string ns);
bool promptConfirm(string str_question);
bool moveNear();
bool openFingers();
bool approachMove(double x, double y, double z);
bool closeFingers();
bool executeTaskTrajectory();
bool updateSpringForces(bool active, double rl, double k);
bool applyForcesGradually();
bool releaseForcesGradually();

int main(int argc, char **argv){

    ros::init(argc, argv, "grasp_node");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // attempt to get ros parameters
    if(!getParams()) return -1;

    // ros communication
    hand_pose_client_ =
      nh.serviceClient<allegro_hand_kdl::PoseRequest>("desired_pose");
    joint_traj_client_ =
      nh.serviceClient<lfd_experiments::TrajectoryRequest>("desired_traj");
    task_traj_client_ =
      nh.serviceClient<lfd_experiments::TrajectoryRequest>("desired_task_traj");
    hand_spring_client_ =
      nh.serviceClient<lfd_experiments::SpringUpdate>("spring_update");
    stiffness_adjuster_client_ =
      nh.serviceClient<std_srvs::SetBool>("adjust_stiffness");
    std_srvs::SetBool stf_srv;
    std_srvs::Empty grip_srv;

    // to publish run experiment
    label_pub_ =
      nh.advertise<std_msgs::String>("experiment_label", 1);

  // Setup move group interface
  move_group_.reset(
    new moveit::planning_interface::MoveGroupInterface(manipulator_name_));
    move_group_->setPlannerId("RRTConnect");

    move_group_->setMaxVelocityScalingFactor(velocity_scaler_);
    move_group_->setMaxAccelerationScalingFactor(velocity_scaler_);

    // Sleep to give time to moveit
    ros::Duration(1.0).sleep();

    // monitor grasp phases
    int phase = 0;
    int action_no;
    ROS_INFO("Grasp node: phase %d", phase);

    while(ros::ok()){
      // rest time for user comprehension
      ros::Duration(0.5).sleep();

      bool done = false;
      switch (phase) {
        case 0:
          // prompt user for the grasp type
          if(!printLibrary("pick")) return false;
          cout<< "Select index:";
          cin >> action_no;
          done = readGraspParams(action_no);

          break;
        case 1:
          // move robot arm near the object
          done = moveNear();
          break;
        case 2:
          // open fingers for pregrasp shape
          done = openFingers();
          // we don't care about success
          done = true;
          break;
        case 3:
          // TODO: user-defined approach direction
        done = approachMove(0, 0, -z_approach_); // meters
          break;
        case 4:
          // close the fingers around the object
        if(promptConfirm("Grasp"))
          done = closeFingers();
        else if(promptConfirm("Skip"))
          done = true;
          break;
        case 5:
          ROS_INFO("Grasp node: Move the fingers to have contact.");
          // complete grasp by applying envelop forces
          if(promptConfirm("Apply forces")){
            done = applyForcesGradually();
            // activate stiffness adjuster for better manipulation
            stf_srv.request.data = true;
            stiffness_adjuster_client_.call(stf_srv);
          }
          break;
        case 6:
          // pick the object up
          done = approachMove(0, 0, z_pick_); // meters
          break;
        case 7:
          // manipulate as many times as user wants
          if(!promptConfirm("Manipulate")){
            phase = 9; // skip phase
            break;
          }
          // choose manipulation action
          printLibrary("manipulate");
          cout<< "Select index:";
          cin >> action_no;
          done = readManipulationParams(action_no);
          break;
        case 8:
          done = executeTaskTrajectory();
          break;
        case 9:
          // release it by stopping envelop forces
          if(promptConfirm("Put down"))
          done = approachMove(0, 0, -z_pick_); // meters
          else // go back to manipulate
            phase = 7;
          break;
        case 10:
          if(promptConfirm("Release")){
            // deactivate stiffness adjuster
            stf_srv.request.data = false;
            stiffness_adjuster_client_.call(stf_srv);
            // release object
            updateSpringForces(true, 1.0, stiffness_/2);
            ros::Duration(0.2).sleep();
            updateSpringForces(true, 1.3, stiffness_/4);
            ros::Duration(0.2).sleep();
            updateSpringForces(false, 1.0, 0.0);
            done = true;
          }
          break;
        case 11:
          ROS_INFO("Grasp node: done.");
          // ask user to grasp again
          if(promptConfirm("Repeat"))
            phase = 0;
          else
            ros::shutdown();
          break;
        // default:
      }
      // update phase if done
      if(done) {
        phase++;
        ROS_INFO("Grasp node: phase %d", phase);
      }
    }

  ROS_INFO("Grasp node: terminating.");
  ros::spinOnce();
  return 0;
}

bool getParams(){

  string param_name;

  // manipulator name
  if(!ros::param::get("~manipulator", manipulator_name_)){
    ROS_WARN("Grasp node: Can't get manipulator param.");
		return false;
  }
  ROS_DEBUG("Grasp node: manipulator is %s.", manipulator_name_.c_str());

  // end link name
  if(!ros::param::get("~end_link", end_link_name_)){
    ROS_WARN("Grasp node: Can't get end_link param.");
		return false;
  }
  ROS_DEBUG("Grasp node: end_link is %s.", end_link_name_.c_str());

  // velocity scaler
  if(!ros::param::get("~velocity_scaler", velocity_scaler_)){
    ROS_WARN("Grasp node: Can't get velocity_scaler param.");
		return false;
  }
  ROS_DEBUG("Grasp node: velocity_scaler is %.3f.", velocity_scaler_);

  // spring rest length
  if(!ros::param::get("~rest_length", rest_length_)){
    ROS_WARN("Grasp node: Can't get rest_length param.");
		return false;
  }
  ROS_DEBUG("Grasp node: rest_length is %.3f.", rest_length_);

  // spring stiffness
  if(!ros::param::get("~stiffness", stiffness_)){
    ROS_WARN("Grasp node: Can't get stiffness param.");
		return false;
  }
  ROS_DEBUG("Grasp node: stiffness is %.3f.", stiffness_);

  // zero reference point in workspace
  if(!ros::param::has("lfd_experiments/grasps/zero_point")){
    ROS_ERROR("Grasp node: Can't get lfd_experiments/grasps/zero_point param.");
		return false;
  }
  ros::param::get("lfd_experiments/grasps/zero_point/x", zero_point_.x);
  ros::param::get("lfd_experiments/grasps/zero_point/y", zero_point_.y);
  ros::param::get("lfd_experiments/grasps/zero_point/z", zero_point_.z);

  // dmp folder path
  if(!ros::param::has("lfd_experiments/grasps/dmp_path")){
    ROS_ERROR("Grasp node: Can't get lfd_experiments/grasps/dmp_path param.");
		return false;
  }
  ros::param::get("lfd_experiments/grasps/dmp_path", dmp_path_);


  return true;

}

bool printLibrary(string ns){

    string grasp_ns = "lfd_experiments/grasps/"+ns;
    if(!ros::param::has(grasp_ns)){
      ROS_WARN("Grasp node: Can't find %s library.", ns.c_str());
      return false;
    }

    cout << ns << " library:" << endl;

    // iterate existing items (following pattern i0,i1...)
    int gi = 0;
    while(ros::param::has(grasp_ns+"/i"+to_string(gi))){
      string name;
      ros::param::get(grasp_ns+"/i"+to_string(gi)+"/name", name);

      cout << gi << ": " << name << endl;

      gi++;
    }

    return true;
}

/* Gets the params of a grasp and assigns them to variables */
bool readGraspParams(int gi){

  string grasp_ns = "lfd_experiments/grasps/pick";
  if(!ros::param::has(grasp_ns+"/i"+to_string(gi))){
    // failed to find
    ROS_WARN("Grasp node: Can't find grasp %d.", gi);
    return false;
  }

  // assign params
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/name", action_name_);

  ROS_INFO("Grasp node: loading grasp %s", action_name_.c_str());

  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/pregrasp", pregrasp_name_);
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/dmp", dmp_name_);

  // palm pose params
    // position
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/palm/pos/x", palm_pose_.position.x);
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/palm/pos/y", palm_pose_.position.y);
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/palm/pos/z", palm_pose_.position.z);
    // orientation
  double rot_x,rot_y,rot_z;
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/palm/rot/x", rot_x);
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/palm/rot/y", rot_y);
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/palm/rot/z", rot_z);
    // create Quaternion from Euler YXZ
  tf2::Quaternion q_tf;
  q_tf.setEuler(rot_y, rot_x, rot_z);
    // assign to target orientation
  palm_pose_.orientation = tf2::toMsg(q_tf);

  return true;

}

/* Gets the params of a manipulation and assigns them to variables */
bool readManipulationParams(int gi){

  string grasp_ns = "lfd_experiments/grasps/manipulate";
  if(!ros::param::has(grasp_ns+"/i"+to_string(gi))){
    // failed to find
    ROS_WARN("Grasp node: Can't find manipulation %d.", gi);
    return false;
  }

  // assign params
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/name", action_name_);

  ROS_INFO("Grasp node: loading manipulation %s", action_name_.c_str());

  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/dmp", dmp_name_);
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/type", manipulate_type_);
  ros::param::get(grasp_ns+"/i"+to_string(gi)+"/time", action_time_);

  // check custom goal
  if(ros::param::has(grasp_ns+"/i"+to_string(gi)+"/goal")){
    ROS_INFO("Grasp node: Manipulating with custom transformation.");

    custom_goal_.reset(new geometry_msgs::Pose());
    // translation
    ros::param::get(grasp_ns+"/i"+to_string(gi)+"/goal/pos/x", custom_goal_->position.x);
    ros::param::get(grasp_ns+"/i"+to_string(gi)+"/goal/pos/y", custom_goal_->position.y);
    ros::param::get(grasp_ns+"/i"+to_string(gi)+"/goal/pos/z", custom_goal_->position.z);
    // rotation
    double rot_x,rot_y,rot_z;
    ros::param::get(grasp_ns+"/i"+to_string(gi)+"/goal/rot/x", rot_x);
    ros::param::get(grasp_ns+"/i"+to_string(gi)+"/goal/rot/y", rot_y);
    ros::param::get(grasp_ns+"/i"+to_string(gi)+"/goal/rot/z", rot_z);
      // create Quaternion from Euler YXZ
    tf2::Quaternion q_tf;
    q_tf.setEuler(rot_y, rot_x, rot_z);
      // assign to target orientation
    custom_goal_->orientation = tf2::toMsg(q_tf);
  }else{
    custom_goal_.reset();
  }

  return true;
}

bool promptConfirm(string str_question){

    string ans;

    cout << str_question << "? (y/n) ";
    cin >> ans;

    // execute if user confirms
    if(ans=="y") return true;
    return false;
}

bool moveArm(geometry_msgs::Pose &target_pose){

  move_group_->setPoseTarget(target_pose, end_link_name_);

  cout << "Grasp: target pose " << target_pose.position.x
          << ", " << target_pose.position.y
          << ", " << target_pose.position.z << "\n";

  // create a plan for this goal
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool possible = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ask for user confirmation before executing a plan
  if(possible){
    // ask user
    if(promptConfirm("Execute")){

      move_group_->execute(plan);
      // Sleep to give time to moveit
      ros::Duration(0.5).sleep();

      return true;
    }
  }

  return false;

}

bool approachMove(double x, double y, double z){

  // set to current
  geometry_msgs::Pose target_pose = move_group_->getCurrentPose(end_link_name_).pose;

  // apply approach movement
  target_pose.position.x += x;
  target_pose.position.y += y;
  target_pose.position.z += z;

  moveArm(target_pose);
}

bool moveNear(){
  // set the goal
  geometry_msgs::Pose target_pose;

  // assign from grasp parameters
  target_pose.orientation = palm_pose_.orientation;
  target_pose.position.x = zero_point_.x + palm_pose_.position.x;
  target_pose.position.y = zero_point_.y + palm_pose_.position.y;
  target_pose.position.z = zero_point_.z + palm_pose_.position.z;

  // hand will move above the object for approach
  target_pose.position.z += z_approach_;

  moveArm(target_pose);
}

bool openFingers(){

  // pregrasp pose for hand
  allegro_hand_kdl::PoseRequest pr_srv;
  pr_srv.request.pose = pregrasp_name_;

  if(!hand_pose_client_.call(pr_srv))
    return false;

  return pr_srv.response.success;
}

bool closeFingers(){

  ros::spinOnce();
  // issue a trajectory service
  lfd_experiments::TrajectoryRequest tr_srv;
  tr_srv.request.dmp = dmp_path_ +"/"+ dmp_name_+".xml";
  tr_srv.request.duration = 12;
  tr_srv.request.custom_goal = false;

  return joint_traj_client_.call(tr_srv);
}

bool executeTaskTrajectory(){

  // publish label for rosbag
  std_msgs::String msg_str;
  msg_str.data = action_name_;
  label_pub_.publish(msg_str);

  // issue a trajectory service
  lfd_experiments::TrajectoryRequest tr_srv;
  tr_srv.request.dmp = dmp_path_ +"/"+ dmp_name_+".xml";
  tr_srv.request.duration = action_time_;
  // assign custom goal if specified
  if(custom_goal_){
    tr_srv.request.custom_goal = true;
    tr_srv.request.cartesian_goal = *custom_goal_;
  }else
    custom_goal_ = false;


  return task_traj_client_.call(tr_srv);
}

/**
 * Handy function to update a spring's parameters
 * It calls the appropriate service
**/
bool updateSpringForces(bool active, double rl, double k){

    // issue a spring service
    lfd_experiments::SpringUpdate su_srv;
    su_srv.request.active = active;
    su_srv.request.rest_length = rl;
    su_srv.request.stiffness = k;
    su_srv.request.index = -1; // -1 means all fingers

    if(!hand_spring_client_.call(su_srv))
      return false;

    return true;
}


// envelop forces are increased slowly, instead of instant activation
bool applyForcesGradually(){

  int count = 100;
  double rl_step = (rest_length_ - 1.0) / count;

  double rl_now = 1.0;

  // force is applied by gradually decreasing the spring rest length
  for(int i=0; i < count; i++){

    rl_now += rl_step;

    bool updated = updateSpringForces(true, rl_now, -1);
    if(!updated) return false;

    ros::Duration(2.0/count).sleep();
    ros::spinOnce();
  }

  return true;
}

bool releaseForcesGradually(){

  int count = 80;
  double rl_step = (1.1 - rest_length_) / count;
  double k_step = 1.0-stiffness_ / count;

  double rl_now = rest_length_;
  double k_now = stiffness_;
  // force is applied by gradually decreasing the spring rest length
  for(int i=0; i < count; i++){

    rl_now += rl_step;
    k_now += k_step;

    bool updated = updateSpringForces(true, rl_now, k_now);
    if(!updated) return false;

    ros::Duration(0.05).sleep();
    ros::spinOnce();
  }

  return true;
}
