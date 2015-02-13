#include "multi_uav_exploration_node.hpp"

#include <memory>

#include "ros/ros.h"

#include "multi_uav_exploration/explorer.hpp"
#include "multi_uav_exploration/relay_explorer.hpp"
#include "multi_uav_exploration/sacrifice_explorer.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "multi_uav_exploration");
  ros::NodeHandle nh("~");

  // Determine the role of this robot
  bool isRelay = false;
  nh.getParam("is_relay", isRelay);

  bool isSacrifice = false;
  nh.getParam("is_sacrifice", isSacrifice);

  if(isRelay && isSacrifice) {
    ROS_ERROR("Cannot be relay and sacrifice explorer");
    ros::shutdown();
  } else if(!isRelay && !isSacrifice) {
    ROS_ERROR("Must be relay or sacrifice explorer");
    ros::shutdown();
  }

  std::unique_ptr<multi_uav_exploration::Explorer> explorer;
  if(isRelay) {
    ROS_INFO("multi_uav_exploration_node started as RELAY");
    explorer = std::make_unique<multi_uav_exploration::RelayExplorer>(nh);
  } else if(isSacrifice) {
    ROS_INFO("multi_uav_exploration_node started as SACRIFICE");
    explorer = std::make_unique<multi_uav_exploration::SacrificeExplorer>(nh);
  }

  ros::Rate rate(10.0);
  while(ros::ok()) {
    explorer->spinOnce();

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
