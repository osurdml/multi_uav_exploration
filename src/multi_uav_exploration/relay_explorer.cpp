#include "multi_uav_exploration/relay_explorer.hpp"

namespace multi_uav_exploration {

RelayExplorer::RelayExplorer(ros::NodeHandle& nh)
  : Explorer(nh),
    moveGoalPublisher(ros::NodeHandle("").advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 5)),
    finishSacrificeServer(nh.advertiseService("finish_sacrifice", &RelayExplorer::sacrificeFinishCallback, this)) {
}

void RelayExplorer::spinOnce() {
  Explorer::spinOnce();

  switch(mode) {
    case BehaviorMode::Initialize:
      break;
    case BehaviorMode::Explore:
      break;
    case BehaviorMode::Relay:
      break;
    case BehaviorMode::Sacrifice:
      break;
    case BehaviorMode::GoHome:
      geometry_msgs::PoseStamped homePose;

      homePose.pose.position.x = homeLocation.getOrigin().getX();
      homePose.pose.position.y = homeLocation.getOrigin().getY();
      homePose.pose.position.z = homeLocation.getOrigin().getZ();

      // TODO: Don't care about rotation?
      homePose.pose.orientation.x = homeLocation.getRotation().getX();
      homePose.pose.orientation.y = homeLocation.getRotation().getY();
      homePose.pose.orientation.z = homeLocation.getRotation().getZ();
      homePose.pose.orientation.w = homeLocation.getRotation().getW();

      moveGoalPublisher.publish(homePose);
      break;
  }
}

void RelayExplorer::transition(BehaviorMode to) {
  Explorer::transition(to);

  switch(mode) {
    case BehaviorMode::Initialize:
      break;
    case BehaviorMode::Explore:
      break;
    case BehaviorMode::Relay:
      break;
    case BehaviorMode::Sacrifice:
      break;
    case BehaviorMode::GoHome:
      break;
  }
}

void RelayExplorer::exploreTimerCallback(const ros::TimerEvent& event) {
  ROS_ASSERT(mode == BehaviorMode::Explore);

  transition(BehaviorMode::Relay);
}

bool RelayExplorer::sacrificeFinishCallback(FinishSacrifice::Request& req, FinishSacrifice::Response& resp) {
  ROS_ASSERT(mode == BehaviorMode::Relay);

  transition(BehaviorMode::GoHome);
  resp.success = true;

  return true;
}

}
