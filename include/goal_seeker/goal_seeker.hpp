#ifndef GOAL_SEEKER_HPP_
#define GOAL_SEEKER_HPP_

#include "geometry_msgs/Point.h"
#include "ros/publisher.h"
#include "ros/service_server.h"
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>
#include <path_planner/setGoalPoint.h>
// #include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <std_msgs/Bool.h>
#include <cmath>

#include <Eigen/Dense>
#include <array>
#include <thread>
#include <chrono>

// #define TAG_POSE_TOPIC "ar_pose_marker"
#define GOAL_TOPIC "goal_position"
#define WAYPOINT_TOPIC "position_hold/trajectory"
#define POSE_TOPIC "motion_reference/pose"
#define ODOMETRY_TOPIC "odometry"
#define CONTROLNODE_SRV "goal_seeker/run"
#define SETGOAL_SRV "goal_seeker/set_goal"
#define SEEKER_HAS_ENDED_TOPIC "goal_seeker/has_ended"

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

  std::array<geometry_msgs::Pose, 40> poses_;

  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Publisher goal_position_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber tag_pose_sub_;
  ros::ServiceServer control_node_srv;
  ros::ServiceServer set_goal_srv;
  ros::Publisher has_ended_pub_;

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

  float searching_area_height_ = 5.0;
  float searching_area_radious_ = 2.5;
  geometry_msgs::Point poi_;

  float ref_angle_;
  int order_index_;
  int modifier_;
  Eigen::Vector3d tag_position_;
  bool target_found_=false;

  bool checkOrientationReached(const float _angle);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &_msg);
  // void tagPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &_msg);
  bool controlNodeSrv(std_srvs::SetBool::Request &_request, std_srvs::SetBool::Response &_response);
  bool filterTagPosition(const Eigen::Vector3d &_tag_position);
  bool setGoalSrv(path_planner::setGoalPoint::Request &_request,
                  path_planner::setGoalPoint::Response &_response);
  void endSearch();
};

trajectory_msgs::MultiDOFJointTrajectoryPoint generateWaypointMsg(const geometry_msgs::Pose &_poses, const float _yaw);
geometry_msgs::PoseStamped generateGoalPoseMsg(const Eigen::Vector3d _goal_position);
geometry_msgs::PoseStamped generatePoseStampedMsg(const geometry_msgs::Pose _waypoint,
                                                  const float _yaw);

Eigen::Vector3d identifyTagOrientation(const Eigen::Vector3d tag_position_);
float identifySeekYaw(const Eigen::Vector3d tag_position_);

#endif // GOAL_SEEKER_HPP_
