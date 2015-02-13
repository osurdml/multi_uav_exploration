#include "multi_uav_exploration/explorer.hpp"

namespace multi_uav_exploration {

Explorer::Explorer(ros::NodeHandle& nh)
  : nh(nh),
    mode(BehaviorMode::Initialize),
    setStateServer(nh.advertiseService("set_state",
          &multi_uav_exploration::Explorer::setStateCallback, this)),
    tfListener(nh) {
}

void Explorer::spinOnce() {
  switch(mode) {
    case BehaviorMode::Initialize:
      // Home location is continually updated to the current location until we
      // advance out of the `Initialize` state.
      try {
        tfListener.lookupTransform("/camera_link", "/map", ros::Time(0), homeLocation);
      } catch(tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
      break;
    case BehaviorMode::Explore:
      // TODO: Frontier exploration for a specified duration
      break;
    case BehaviorMode::Relay:
      break;
    case BehaviorMode::Sacrifice:
      break;
    case BehaviorMode::GoHome:
      break;
  }
}

void Explorer::transition(BehaviorMode to) {
  ROS_INFO("Moving from state %d to %d", mode, to);

  mode = to;

  switch(mode) {
    case BehaviorMode::Initialize:
      break;
    case BehaviorMode::Explore:
      // When entering the explore mode, set a timer to impose an artificial
      // battery constraint.
      nh.createTimer(ros::Duration(10.0), &Explorer::exploreTimerCallback, this);
      break;
    case BehaviorMode::Relay:
      break;
    case BehaviorMode::Sacrifice:
      break;
    case BehaviorMode::GoHome:
      break;
  }
}

bool Explorer::setStateCallback(SetState::Request& req, SetState::Response& resp) {
  // TODO: Do some verification that this is a valid state transition.
  transition(static_cast<BehaviorMode>(req.state));
  resp.success = true;

  return true;
}

}
