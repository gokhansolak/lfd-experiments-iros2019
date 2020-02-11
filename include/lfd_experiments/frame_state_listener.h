// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#ifndef FRAME_STATE_LISTENER_H_
#define FRAME_STATE_LISTENER_H_

#include <ros/ros.h>

#include <kdl/kdl.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <dmp_tools/state_listener.h>

#include <geometry_msgs/PoseArray.h>

#include <spring_framework/spring_control.h>

using namespace ros_dmp_tools;
using namespace spring_framework;

namespace lfd_experiments{
const std::string POSE_ARRAY_TOPIC = "pose_array";

// StateListener implementation for virtual object frame
// StateListener is a part of dmp_tools package
class FrameStateListener: public StateListener{

protected:

	spring_framework::SpringController * springs_;

	ros::Timer timer_;

	// ros params
	int finger_count_;
	int finger_length_;

	bool getParams_(const ros::NodeHandle& nh);
	void subscriberCallback_(const geometry_msgs::PoseArray::ConstPtr &msg);

public:
	FrameStateListener(ros::NodeHandle* nh);
  ~FrameStateListener();

  virtual void startListening();
	
};
} // end namespace lfd_experiments


#endif /* FRAME_STATE_LISTENER_H_*/
