// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include <lfd_experiments/frame_state_listener.h>

using namespace lfd_experiments;
using namespace std;

/*********************************************************************
* Comment
*********************************************************************/
FrameStateListener::FrameStateListener(ros::NodeHandle* nh):
StateListener(nh)
{
  // get ros params
  if(!getParams_(*nh)) return;

  // just to calculate virtual frame
  springs_ = new SpringController(finger_count_);
}
/*********************************************************************
* Comment
*********************************************************************/
FrameStateListener::~FrameStateListener()
{
  delete springs_;
}
/*********************************************************************
* Comment
*********************************************************************/
void FrameStateListener::startListening()
{

  sub_ = nh_->subscribe<geometry_msgs::PoseArray>(POSE_ARRAY_TOPIC, 1,
                              &FrameStateListener::subscriberCallback_, this);

}
/*********************************************************************
* Comment
*********************************************************************/
void FrameStateListener::subscriberCallback_(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  // convert finger poses to kdl frames
  vector<KDL::Frame> ff_vec(finger_count_);
  for(int fi=0; fi < finger_count_; fi++){
    tf::poseMsgToKDL(msg->poses[fi], ff_vec[fi]);
  }

  // update virtual frame
  springs_->updateState(ff_vec);

  // get virtual frame
  KDL::Frame vf;
  springs_->getVirtualFrame(vf);

  // ********* create message
  // create a double vector as RecordState data
  vector<double> rs;

  // ***** position
  for(int di=0; di<3; di++)
    rs.push_back(vf.p(di));

  // ***** orientation
  // we record only the r_x and r_z vectors since r_y = r_z * r_x
  KDL::Vector r_x = vf.M.UnitX();
  for(int di=0; di<3; di++)
    rs.push_back(r_x(di));

  KDL::Vector r_z = vf.M.UnitZ();
  for(int di=0; di<3; di++)
    rs.push_back(r_z(di));

  // publish state for trajectory recorder
  publishRecordState(rs);

}

bool FrameStateListener::getParams_(const ros::NodeHandle & nh){
    string param_name;

    if(!nh.searchParam("robot_hand/finger_count", param_name) ){
      ROS_ERROR("FingertipStateListener: Can't find finger_count param.");
      return false;
    }

    ros::param::get(param_name, finger_count_);

    if(!nh.searchParam("robot_hand/finger_length", param_name) ){
      ROS_ERROR("FingertipStateListener: Can't find finger_length param.");
      return false;
    }

    ros::param::get(param_name, finger_length_);

    return true;
}
