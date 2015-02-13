#pragma once

#include "tf/transform_listener.h"

#include "multi_uav_exploration/SetState.h"

namespace multi_uav_exploration {

enum BehaviorMode {
  /**
   * Initialize the home position.
   */
  Initialize = SetState::Request::STATE_INITIALIZE,

  /**
   * Do frontier exploration.
   */
  Explore = SetState::Request::STATE_EXPLORE,

  /**
   * Wait for a message from a sacrificial robot.
   */
  Relay = SetState::Request::STATE_RELAY,

  /**
   * Continue exploring beyond the minimum battery boundary (to return home).
   */
  Sacrifice = SetState::Request::STATE_SACRIFICE,

  /**
   * Navigate to the home location.
   */
  GoHome = SetState::Request::STATE_GOHOME
};

class Explorer {
public:
  Explorer(ros::NodeHandle& nh);

  virtual void spinOnce();
  virtual void transition(BehaviorMode to);

  bool setStateCallback(SetState::Request& req, SetState::Response& resp);
  virtual void exploreTimerCallback(const ros::TimerEvent& event) = 0;

protected:
  ros::NodeHandle& nh;

  BehaviorMode mode;

  ros::ServiceServer setStateServer;

  tf::TransformListener tfListener;
  tf::StampedTransform homeLocation;
};

}
