// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


/**
  * This node constantly monitors optoforce tactile data
  * and ensures that it stays in a desired boundary by
  * adjusting the spring stiffnesses.
  * Spring stiffnesses are contolled by issuing SpringUpdate
  * service calls to the spring trajectory service node.
  *
  * -- Gokhan
**/

#include <signal.h>
#include <boost/bind.hpp>
#include <ros/ros.h>

#include <allegro_hand_kdl/allegro_kdl_config.h>

#include <geometry_msgs/WrenchStamped.h>
#include <lfd_experiments/SpringUpdate.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

ros::ServiceClient spring_client_;
ros::Publisher stiffness_pub_; // for debug TODO: make optional
ros::Publisher magnitude_pub_; // for debug TODO: make optional

bool active_ = false;

// params
double target_force_;
double k_init_;
vector<double> k_last_;

double gain_ = 0.0000;

vector<double> force_last_;

bool getParams();
void tactileForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, int index);
bool startRequest(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
void sigintCallback(int sig);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adjust_stiffness");

    ros::NodeHandle nh;

    ROS_INFO("Adjust stiffness node is ready.");


    // attempt to get ros parameters
    if(!getParams()) return -1;

    // create tactile data subscribers
    vector<ros::Subscriber> tactile_subs;
    for(int fi=0; fi<allegro_hand_kdl::FINGER_COUNT; fi++)
      tactile_subs.push_back(
        nh.subscribe<geometry_msgs::WrenchStamped>( "/optoforce_wrench_"+to_string(fi), 1, boost::bind(tactileForceCallback, _1, fi))
      );

    // create spring service client
    spring_client_ =
      nh.serviceClient<lfd_experiments::SpringUpdate>("spring_update");

    // create stiffness publisher for debug
    stiffness_pub_ =
      nh.advertise<std_msgs::Float64MultiArray>("stiffness_adjusted", 2);
    magnitude_pub_ =
      nh.advertise<std_msgs::Float64MultiArray>("tactile_magnitude", 2);

    // restart server for reactivating fingers
    ros::ServiceServer serv = nh.advertiseService("adjust_stiffness", startRequest);

    ros::spin();

    return 0;
}

bool getParams(){

  string param_name;

  // target force for all fingers (thumb tripled)
  if(!ros::param::get("~target_force", target_force_)){
    ROS_WARN("Adjust stiffness: Can't get target_force param.");
		return false;
  }
  ROS_DEBUG("Adjust stiffness: target_force is %.3f.", target_force_);

  // initial stiffness
  if(!ros::param::get("~stiffness", k_init_)){
    ROS_WARN("Adjust stiffness: Can't get stiffness param.");
    return false;
  }
  ROS_DEBUG("Adjust stiffness: stiffness is %.3f.", k_init_);

  // init k_last_
  k_last_.clear();
  k_last_.resize(allegro_hand_kdl::FINGER_COUNT, k_init_);
  force_last_.clear();
  force_last_.resize(allegro_hand_kdl::FINGER_COUNT, k_init_);


  // gain
  if(!ros::param::get("~gain", gain_)){
    ROS_WARN("Adjust stiffness: Can't get gain param.");
    return false;
  }
  ROS_DEBUG("Adjust stiffness: gain is %.5f.", gain_);

  return true;
}

// Tactile callback is called with different index for each finger
// it only reads the tactile force amplitude for a finger
// then the stiffness of that finger is adjusted if necessary
void tactileForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, int index) {

  // needs startRequest to be activated
  if(!active_) return;

  // calculate norm and assign to the vector
  force_last_[index] = pow(msg->wrench.force.x, 2);
  force_last_[index] += pow(msg->wrench.force.y, 2);
  force_last_[index] += pow(msg->wrench.force.z, 2);
  force_last_[index] = sqrt(force_last_[index]);

  // Update springs proportionally by the difference

  // thumb force is scaled
  double f_target = target_force_;
  if(index==3) f_target = 3 * target_force_;

  // update proportional to difference
  double adjustment = (f_target-force_last_[index]) * gain_;

  // if adjustment is significant, then call update service

    k_last_[index] += adjustment;

    // limit current k
    k_last_[index] = max(k_last_[index], k_init_*0.0);
    k_last_[index] = min(k_last_[index], k_init_*5.0);

    lfd_experiments::SpringUpdate su_srv;
    su_srv.request.active = true;
    su_srv.request.rest_length = -1;
    su_srv.request.stiffness = k_last_[index];
    su_srv.request.index = index;

      // call & communicate
      spring_client_.call(su_srv);
      ros::spinOnce();

      ROS_DEBUG("Adjust stiffness: f %d k= %.3f", index, k_last_[index]);


  if(index == 3){
    // publish latest stiffness for debug (for all fingers)
    std_msgs::Float64MultiArray msg_debug;
    for(int fi=0; fi<allegro_hand_kdl::FINGER_COUNT; fi++)
      msg_debug.data.push_back(k_last_[fi]);
    stiffness_pub_.publish(msg_debug);
    // then publish the force magnitude
    msg_debug.data.clear();
    for(int fi=0; fi<allegro_hand_kdl::FINGER_COUNT; fi++)
      msg_debug.data.push_back(force_last_[fi]);
    magnitude_pub_.publish(msg_debug);
  }

}

// Starts the gradual grip process, if another one is not in progress
bool startRequest(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){

  active_ = request.data;

  ROS_INFO("Adjust stiffness: activity %d.", active_);

  // refresh k_last_ to init
  k_last_.clear();
  k_last_.resize(allegro_hand_kdl::FINGER_COUNT, k_init_);

  // initially activate springs at full rest
  lfd_experiments::SpringUpdate su_srv;
  su_srv.request.active = active_;
  su_srv.request.rest_length = -1;
  su_srv.request.stiffness = k_init_;
  su_srv.request.index = -1; // -1 means all fingers
    // call & communicate
    spring_client_.call(su_srv);
    ros::spinOnce();


  return true;
}
