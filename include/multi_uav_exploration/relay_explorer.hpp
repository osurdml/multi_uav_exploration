#pragma once

#include "multi_uav_exploration/relay_explorer.hpp"

#include "geometry_msgs/PoseStamped.h"

#include "multi_uav_exploration/explorer.hpp"
#include "multi_uav_exploration/FinishSacrifice.h"

namespace multi_uav_exploration {

class RelayExplorer : public Explorer {
public:
  RelayExplorer(ros::NodeHandle& nh);

  virtual void spinOnce() override;
  virtual void transition(BehaviorMode to) override;

  virtual void exploreTimerCallback(const ros::TimerEvent& event) override;
  bool sacrificeFinishCallback(FinishSacrifice::Request& req, FinishSacrifice::Response& resp);

private:
  ros::Publisher moveGoalPublisher;
  ros::ServiceServer finishSacrificeServer;
};

}
