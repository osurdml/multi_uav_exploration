#pragma once

#include "multi_uav_exploration/sacrifice_explorer.hpp"

#include "multi_uav_exploration/explorer.hpp"
#include "multi_uav_exploration/FinishSacrifice.h"

namespace multi_uav_exploration {

class SacrificeExplorer : public Explorer {
public:
  SacrificeExplorer(ros::NodeHandle& nh);

  virtual void process(const ros::TimerEvent& event) override;
  virtual void transition(BehaviorMode to) override;

  virtual void exploreTimerCallback(const ros::TimerEvent& event) override;
  void sacrificeTimerCallback(const ros::TimerEvent& event);

private:
  ros::ServiceClient finishSacrificeClient;
};

}
