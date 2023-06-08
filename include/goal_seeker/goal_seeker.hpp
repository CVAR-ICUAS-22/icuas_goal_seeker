#ifndef GOAL_SEEKER_HPP_
#define GOAL_SEEKER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "image_transport/subscriber.h"
#include "ros/publisher.h"
#include "ros/service_server.h"
#include <opencv2/core/types.hpp>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include <nav_msgs/Odometry.h>
#include <path_planner/setGoalPoint.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cmath>
#include <thread>

#define GOAL_TOPIC "goal_position"
#define WAYPOINT_TOPIC "position_hold/trajectory"
#define POSE_TOPIC "motion_reference/pose"
#define ODOMETRY_TOPIC "odometry"
#define CONTROLNODE_SRV "goal_seeker/run"
#define SETGOAL_SRV "goal_seeker/set_goal"
#define SEEKER_HAS_ENDED_TOPIC "goal_seeker/has_ended"
#define RUN_DETECTION_TOPIC "/crack_detector/run"
#define MAP_TOPIC "ego_map"
#define PI 3.14159265
#define N_POSES 20

class GoalSeeker {
public:
  GoalSeeker();
  ~GoalSeeker();
  void run();
  void start();
  void stop();

  std::array<geometry_msgs::Pose, N_POSES> poses_;

  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Publisher goal_position_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher run_detection_pub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber tag_pose_sub_;
  ros::ServiceServer control_node_srv_;
  ros::ServiceServer set_goal_srv_;
  ros::Publisher has_ended_pub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber map_sub_;

  nav_msgs::Odometry odometry_;
  geometry_msgs::Point poi_;
  cv::Mat map_;

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
  bool find_nearest_wall_ = false;

  float search_area_height_ = 5.0;
  float search_area_radious_ = 2.5;

  float ref_angle_;
  int order_index_;
  int modifier_;
  Eigen::Vector3d tag_position_;
  bool target_found_ = false;
  std::set<float> near_walls_yaw_;
  std::set<float> yaw_visited_;

  bool checkOrientationReached(const float _angle);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &_msg);
  // void tagPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr
  // &_msg);
  bool controlNodeSrv(std_srvs::SetBool::Request &_request,
                      std_srvs::SetBool::Response &_response);
  bool filterTagPosition(const Eigen::Vector3d &_tag_position);
  bool setGoalSrv(path_planner::setGoalPoint::Request &_request,
                  path_planner::setGoalPoint::Response &_response);
  void endSearch();
  void mapCallback(const sensor_msgs::ImageConstPtr &_map);
  bool cellIsOccupied(const cv::Point2i &_cell);
  bool findYawOfInterest(float &_yoi);
  bool findYawsOfInterest(std::vector<float> &_yoi);
  float calculateYaw(cv::Point2i &_cell, cv::Point2i &_center);
  void runDetector(bool _run_node);
};

trajectory_msgs::MultiDOFJointTrajectoryPoint
generateWaypointMsg(const geometry_msgs::Pose &_poses, const float _yaw);
geometry_msgs::PoseStamped
generateGoalPoseMsg(const Eigen::Vector3d _goal_position);
geometry_msgs::PoseStamped
generatePoseStampedMsg(const geometry_msgs::Pose _waypoint, const float _yaw);

Eigen::Vector3d identifyTagOrientation(const Eigen::Vector3d tag_position_);
float identifySeekYaw(const Eigen::Vector3d tag_position_);

#endif // GOAL_SEEKER_HPP_
