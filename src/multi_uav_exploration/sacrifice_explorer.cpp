#include "multi_uav_exploration/sacrifice_explorer.hpp"

namespace multi_uav_exploration {

SacrificeExplorer::SacrificeExplorer(ros::NodeHandle& nh)
  : Explorer(nh),
    finishSacrificeClient(nh.serviceClient<FinishSacrifice>("/finish_sacrifice")) {
}

void SacrificeExplorer::spinOnce() {
  Explorer::spinOnce();

  switch(mode) {
    case BehaviorMode::Initialize:
      break;
    case BehaviorMode::Explore:
      break;
    case BehaviorMode::Relay:
      break;
    case BehaviorMode::Sacrifice:
      // TODO: Frontier exploration for a specified duration (2)
      // TODO: Send a FinishSacrifice service call
      break;
    case BehaviorMode::GoHome:
      break;
  }
}

void SacrificeExplorer::transition(BehaviorMode to) {
  Explorer::transition(to);

  switch(mode) {
    case BehaviorMode::Initialize:
      break;
    case BehaviorMode::Explore:
      break;
    case BehaviorMode::Relay:
      break;
    case BehaviorMode::Sacrifice:
      // When entering the sacrifice mode, set a timer to impose an artificial
      // battery constraint.
      nh.createTimer(ros::Duration(10.0), &SacrificeExplorer::sacrificeTimerCallback, this);
      break;
    case BehaviorMode::GoHome:
      break;
  }
}

void SacrificeExplorer::exploreTimerCallback(const ros::TimerEvent& event) {
  ROS_ASSERT(mode == BehaviorMode::Explore);

  transition(BehaviorMode::Sacrifice);
}

void SacrificeExplorer::sacrificeTimerCallback(const ros::TimerEvent& event) {
  ROS_ASSERT(mode == BehaviorMode::Sacrifice);

  FinishSacrifice finish;
  finishSacrificeClient.call(finish);
}

}
