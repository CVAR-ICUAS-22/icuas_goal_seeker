#include "goal_seeker.hpp"

GoalSeeker::GoalSeeker()
{
  odometry_sub_ = nh_.subscribe(ODOMETRY_TOPIC, 1, &GoalSeeker::odometryCallback, this);
  tag_pose_sub_ = nh_.subscribe(TAG_POSE_TOPIC, 1, &GoalSeeker::tagPoseCallback, this);
  waypoint_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(WAYPOINT_TOPIC, 5);
  goal_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(GOAL_TOPIC, 5);
  control_node_srv = nh_.advertiseService(CONTROLNODE_SRV, &GoalSeeker::controlNodeSrv, this);

  float seek_distance, seek_height, seek_x_max, seek_x_min, seek_y_max;

  nh_.getParam("goal_seeker/seek_distance", seek_distance);
  nh_.getParam("goal_seeker/seek_height", seek_height);
  nh_.getParam("goal_seeker/seek_start", seek_start_);
  nh_.getParam("goal_seeker/seek_x_max", seek_x_max);
  nh_.getParam("goal_seeker/seek_x_min", seek_x_min);
  nh_.getParam("goal_seeker/seek_y_max", seek_y_max);
  nh_.getParam("goal_seeker/next_point_reached_dist", next_point_reached_dist_);
  nh_.getParam("goal_seeker/next_point_reached_yaw", next_point_reached_yaw_);
  nh_.getParam("goal_seeker/inspection_distance", inspection_distance_);
  nh_.getParam("goal_seeker/inspection_height", inspection_height_);
  nh_.getParam("goal_seeker/end_inspection_tag_position_diff", end_inspection_tag_position_diff);
  
  ROS_INFO("Seek distance: %.2f", seek_distance);
  ROS_INFO("Seek height: %.2f", seek_height);
  ROS_INFO("Seek start: %s", seek_start_.c_str());
  ROS_INFO("Seek x max: %.2f", seek_x_max);
  ROS_INFO("Seek x min: %.2f", seek_x_min);
  ROS_INFO("Seek y max: %.2f", seek_y_max);
  ROS_INFO("Next point reached distance: %.2f", next_point_reached_dist_);
  ROS_INFO("Next point reached yaw: %.2f", next_point_reached_yaw_);
  ROS_INFO("Inspection distance: %2f", inspection_distance_);
  ROS_INFO("Inspection height: %.2f", inspection_height_);
  ROS_INFO("Tag position diff: %.4f", end_inspection_tag_position_diff);

  waypoint_sent_ = false;

  float y_position = seek_y_max - seek_distance;
  float x_max = seek_x_max - seek_distance;

  std::array<std::array<float, 4>, 6>
      seek_points{{{seek_x_min, y_position, seek_height, M_PI_2},
                   {x_max, y_position, seek_height, M_PI_2},
                   {x_max, y_position, seek_height, 0.0},
                   //  {8.0, 1.5, height, 0.0},
                   //  {8.0, -1.5, height, 0.0},
                   {x_max, -y_position, seek_height, 0.0},
                   {x_max, -y_position, seek_height, -M_PI_2},
                   {seek_x_min, -y_position, seek_height, -M_PI_2}}};

  for (int i = 0; i < seek_points.size(); i++)
  {
    poses_[i].position.x = seek_points[i][0];
    poses_[i].position.y = seek_points[i][1];
    poses_[i].position.z = seek_points[i][2];
    // poses_[i].orientation.x = seek_points[i][3];
    // poses_[i].orientation.y = seek_points[i][4];
    // poses_[i].orientation.z = seek_points[i][5];
    poses_[i].orientation.w = seek_points[i][3]; // YAW
  }
}

GoalSeeker::~GoalSeeker()
{
}

void GoalSeeker::start()
{
  ROS_INFO("Node started");
  if (seek_start_ == "best")
  {
  if (odometry_.pose.pose.position.y > 0)
  {
    seek_start_ = "left";
  }
  else
  {
    seek_start_ = "right";
  }
  }

  if (seek_start_ == "left")
  {
    order_index_ = 0;
    modifier_ = 1;
  }

  if (seek_start_ == "right")
  {
    order_index_ = poses_.size() - 1;
    modifier_ = -1;
  }

  ROS_DEBUG("Starting index: %s", seek_start_.c_str());

  run_node_ = true;
}

void GoalSeeker::stop()
{
  run_node_ = false;
  ROS_INFO("Node stopped");
}

void GoalSeeker::run()
{
  const float height = 2.0;

  if (!run_node_)
  {
    return;
  }

  if (send_goal_position_)
  {
    goal_position_pub_.publish(generateGoalPoseMsg(tag_position_));
    ROS_INFO("Goal position: %f, %f, %f", tag_position_.x(), tag_position_.y(), tag_position_.z());
    // send_goal_position_ = false;
  }

  if (!odometry_received_)
  {
    ROS_INFO_ONCE("Waiting for odometry...");
    return;
  }

  // SEEK
  if (!tag_pose_received_)
  {
    if (waypoint_sent_)
    {
      double distance = sqrt(pow(poses_[order_index_].position.x - odometry_.pose.pose.position.x, 2) + pow(poses_[order_index_].position.y - odometry_.pose.pose.position.y, 2));
      if (distance < next_point_reached_dist_)
      {
        if (checkOrientationReached(ref_angle_))
        {
          waypoint_sent_ = false;
          order_index_ += modifier_;
          if (order_index_ == (poses_.size() - 1) || order_index_ == 0)
          {
            modifier_ *= -1;
          }
        }
      }
    }
    else
    {
      ref_angle_ = poses_[order_index_].orientation.w;
      waypoint_pub_.publish(generateWaypointMsg(poses_[order_index_], ref_angle_));
      waypoint_sent_ = true;
      ROS_DEBUG("Sending waypoint %f, %f, %f. Angle %f", poses_[order_index_].position.x, poses_[order_index_].position.y, poses_[order_index_].position.z, ref_angle_);
    }
  }

  // INSPECTION
  if (tag_pose_received_)
  {
    geometry_msgs::Pose seek_pose_;
    Eigen::Vector3d seek_vector = identifyTagOrientation(tag_position_);
    float inspection_distance = inspection_distance_; // in meters
    seek_pose_.position.x = tag_position_.x() + seek_vector.x() * inspection_distance;
    seek_pose_.position.y = tag_position_.y() + seek_vector.y() * inspection_distance;
    seek_pose_.position.z = tag_position_.z() + inspection_height_;

    ref_angle_ = identifySeekYaw(tag_position_);

    // ROS_INFO("INSPECTION POINT SENDED");

    waypoint_pub_.publish(generateWaypointMsg(seek_pose_, ref_angle_));
  }
}

bool GoalSeeker::checkOrientationReached(const float _angle)
{
  tf2::Quaternion q(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  if (fabs(_angle - yaw) < 0.5)
  {
    return true;
  }
  return false;
}

bool GoalSeeker::controlNodeSrv(std_srvs::SetBool::Request &_request, std_srvs::SetBool::Response &_response)
{
  if (run_node_ == _request.data)
  {
    _response.success = false;
    _response.message = "Run node already set to " + std::to_string(run_node_);
    return true;
  }

  _response.success = true;
  _response.message = "Run node set to " + std::to_string(run_node_);

  if (_request.data)
  {
    start();
  }
  else
  {
    stop();
  }

  return true;
}

void GoalSeeker::odometryCallback(const nav_msgs::Odometry::ConstPtr &_msg)
{
  odometry_received_ = true;
  odometry_ = *_msg;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint generateWaypointMsg(const geometry_msgs::Pose &_poses, const float _yaw)
{
  // convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, _yaw);

  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
  trajectory_point.transforms.resize(1);
  trajectory_point.transforms[0].translation.x = _poses.position.x;
  trajectory_point.transforms[0].translation.y = _poses.position.y;
  trajectory_point.transforms[0].translation.z = _poses.position.z;
  trajectory_point.transforms[0].rotation.x = q.x();
  trajectory_point.transforms[0].rotation.y = q.y();
  trajectory_point.transforms[0].rotation.z = q.z();
  trajectory_point.transforms[0].rotation.w = q.w();
  trajectory_point.velocities.resize(1);
  // trajectory_point.velocities[0].x = 0;
  // trajectory_point.velocities[0].y = 0;
  // trajectory_point.velocities[0].z = 0;
  trajectory_point.accelerations.resize(1);
  // trajectory_point.accelerations[0].x = 0;
  // trajectory_point.accelerations[0].y = 0;
  // trajectory_point.accelerations[0].z = 0;
  trajectory_point.time_from_start = ros::Duration(0.1);
  return trajectory_point;
}

void GoalSeeker::tagPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &_msg)
{
  if (_msg->markers.size() == 0)
  {
    return;
  }

  if (!run_node_)
  {
    return;
  }

  if (send_goal_position_)
    return;

  Eigen::Vector3d new_tag_position;
  new_tag_position[0] = _msg->markers[0].pose.pose.position.x;
  new_tag_position[1] = _msg->markers[0].pose.pose.position.y;
  new_tag_position[2] = _msg->markers[0].pose.pose.position.z;

  send_goal_position_ = filterTagPosition(new_tag_position);

  // ROS_INFO("TAG POSITION RECEIVED");

  tag_pose_received_ = true;
}

bool GoalSeeker::filterTagPosition(const Eigen::Vector3d &_tag_position)
{
  Eigen::Vector3d tag_position_diff = _tag_position - tag_position_;
  // ROS_INFO("TAG POSITION DIFF: %f, %f, %f", tag_position_diff[0], tag_position_diff[1], tag_position_diff[2]);
  ROS_INFO("TAG POSITION DIFF MAG: %f", tag_position_diff.norm());
  if (tag_position_diff.norm() > end_inspection_tag_position_diff)
  {
    tag_position_ = _tag_position;
    return false;
  }
  return true;
}

Eigen::Vector3d identifyTagOrientation(const Eigen::Vector3d tag_position_)
{
  float x_max = 12.5f;
  float y_min = 7.5f;
  float confidence = 0.5f;

  if (tag_position_.x() > (x_max - confidence))
  {
    return Eigen::Vector3d(-1, 0, 0);
  }
  else if (abs(tag_position_.y()) < (y_min - confidence))
  {
    return Eigen::Vector3d(-1, 0, 0);
  }

  if (tag_position_.y() > 0)
  {
    return Eigen::Vector3d(0, -1, 0);
  }
  else
  {
    return Eigen::Vector3d(0, 1, 0);
  }
}

float identifySeekYaw(const Eigen::Vector3d tag_position_)
{
  float x_max = 12.5f;
  float y_min = 7.5f;
  float confidence = 0.5f;

  if (tag_position_.x() > (x_max - confidence))
  {
    return 0.0f;
  }
  else if (abs(tag_position_.y()) < (y_min - confidence))
  {
    return 0.0f;
  }

  if (tag_position_.y() > 0)
  {
    return M_PI_2;
  }
  else
  {
    return -M_PI_2;
  }
}

geometry_msgs::PoseStamped generateGoalPoseMsg(const Eigen::Vector3d _goal_position)
{
  geometry_msgs::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = "world";
  goal_pose_msg.header.stamp = ros::Time::now();
  goal_pose_msg.pose.position.x = _goal_position.x();
  goal_pose_msg.pose.position.y = _goal_position.y();
  goal_pose_msg.pose.position.z = _goal_position.z();
  goal_pose_msg.pose.orientation.x = 0.0f;
  goal_pose_msg.pose.orientation.y = 0.0f;
  goal_pose_msg.pose.orientation.z = 0.0f;
  goal_pose_msg.pose.orientation.w = 1.0f;
  return goal_pose_msg;
}
