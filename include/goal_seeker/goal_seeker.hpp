#ifndef GOAL_SEEKER_HPP_
#define GOAL_SEEKER_HPP_

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <Eigen/Dense>
#include <array>
#include <thread>
#include <chrono>

#define TAG_POSE_TOPIC "ar_pose_marker"
#define GOAL_TOPIC "goal_position"
#define WAYPOINT_TOPIC "position_hold/trajectory"
#define ODOMETRY_TOPIC "odometry"
#define CONTROLNODE_SRV "goal_seeker/run"
#define PI 3.14159265
// #define DISTPOINT_TH 0.2
// #define SEEK_DISTANCE 2.5f
// #define INSPECTION_DISTANCE 4.0f
// #define TAG_POSITION_DIFF 0.005f
// #define X_MAX 12.5f
// #define Y_MAX 7.5f

class GoalSeeker
{
public:
  GoalSeeker();
  ~GoalSeeker();
  void run();
  void start();
  void stop();

  std::array<geometry_msgs::Pose, 6> poses_;

  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Publisher goal_position_pub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber tag_pose_sub_;
  ros::ServiceServer control_node_srv;

  nav_msgs::Odometry odometry_;

  bool waypoint_sent_ = false;
  bool odometry_received_ = false;
  bool tag_pose_received_ = false;
  bool run_node_ = false;
  bool send_goal_position_ = false;

  std::string seek_start_;
  float inspection_distance_;
  float inspection_height_;
  float next_point_reached_dist_;
  float next_point_reached_yaw_;
  float end_inspection_tag_position_diff;

  float ref_angle_;
  int order_index_;
  int modifier_;
  Eigen::Vector3d tag_position_;

  bool checkOrientationReached(const float _angle);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &_msg);
  void tagPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &_msg);
  bool controlNodeSrv(std_srvs::SetBool::Request &_request, std_srvs::SetBool::Response &_response);
  bool filterTagPosition(const Eigen::Vector3d &_tag_position);
};

trajectory_msgs::MultiDOFJointTrajectoryPoint generateWaypointMsg(const geometry_msgs::Pose &_poses, const float _yaw);
geometry_msgs::PoseStamped generateGoalPoseMsg(const Eigen::Vector3d _goal_position);

Eigen::Vector3d identifyTagOrientation(const Eigen::Vector3d tag_position_);
float identifySeekYaw(const Eigen::Vector3d tag_position_);

#endif // GOAL_SEEKER_HPP_
