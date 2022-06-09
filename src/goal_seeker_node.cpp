#include "goal_seeker.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_seeker_node");

  GoalSeeker node;

  ros::Rate rate(100);
  while (ros::ok())
  {
    // updating all the ros msgs
    ros::spinOnce();
    // running the node
    node.run();
    rate.sleep();
  }
  ros::shutdown();

  return 0;
}