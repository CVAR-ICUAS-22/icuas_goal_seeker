#include "goal_seeker.hpp"
#include <algorithm>
#include <cstddef>

GoalSeeker::GoalSeeker() : it_(nh_) {
  odometry_sub_ =
      nh_.subscribe(ODOMETRY_TOPIC, 1, &GoalSeeker::odometryCallback, this);
  map_sub_ = it_.subscribe(MAP_TOPIC, 1, &GoalSeeker::mapCallback, this);
  // tag_pose_sub_ = nh_.subscribe(TAG_POSE_TOPIC, 1,
  // &GoalSeeker::tagPoseCallback, this);
  waypoint_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
      WAYPOINT_TOPIC, 5);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);
  has_ended_pub_ = nh_.advertise<std_msgs::Bool>(SEEKER_HAS_ENDED_TOPIC, 1);
  run_detection_pub_ = nh_.advertise<std_msgs::Bool>(RUN_DETECTION_TOPIC, 1);

  goal_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(GOAL_TOPIC, 5);
  control_node_srv_ =
      nh_.advertiseService(CONTROLNODE_SRV, &GoalSeeker::controlNodeSrv, this);
  set_goal_srv_ =
      nh_.advertiseService(SETGOAL_SRV, &GoalSeeker::setGoalSrv, this);

  // float search_radious, search_height;

  nh_.getParam("goal_seeker/search_radious", search_area_radious_);
  nh_.getParam("goal_seeker/search_height", search_area_height_);
  nh_.getParam("goal_seeker/seek_start", seek_start_);
  nh_.getParam("goal_seeker/next_point_reached_dist", next_point_reached_dist_);
  nh_.getParam("goal_seeker/next_point_reached_yaw", next_point_reached_yaw_);
  // nh_.getParam("goal_seeker/inspection_distance", inspection_distance_);
  // nh_.getParam("goal_seeker/inspection_height", inspection_height_);

  ROS_INFO("Search radious: %.2f", search_area_radious_);
  ROS_INFO("Search height: %.2f", search_area_height_);
  ROS_INFO("Seek start: %s", seek_start_.c_str());
  ROS_INFO("Next point reached distance: %.2f", next_point_reached_dist_);
  ROS_INFO("Next point reached yaw: %.2f", next_point_reached_yaw_);
  // ROS_INFO("Inspection distance: %2f", inspection_distance_);
  // ROS_INFO("Inspection height: %.2f", inspection_height_);

  waypoint_sent_ = false;
}

GoalSeeker::~GoalSeeker() {}

void GoalSeeker::start() {
  ROS_INFO("Node started");
  // if (seek_start_ == "best")
  // {
  //   if (odometry_.pose.pose.position.y > 0)
  //   {
  //     seek_start_ = "left";
  //   }
  //   else
  //   {
  //     seek_start_ = "right";
  //   }
  // }

  if (seek_start_ == "top") {
    ROS_INFO("Seek start: DOWNWARDS");
    order_index_ = 0;
    modifier_ = 1;
  } else {
    ROS_INFO("Seek start: UPWARDS");
    order_index_ = poses_.size() - 1;
    modifier_ = -1;
  }

  // ROS_DEBUG("Starting index: %s", seek_start_.c_str());
  // modifier_ = 1;
  // order_index_ = 0;

  // Generate searching waypoints
  std::array<std::array<float, 4>, N_POSES> seek_points;

  // std::array<double, 4> initial_pose{{poi_.x, poi_.y, poi_.z +
  // search_area_height_}}; std::array<double, 4> initial_pose{{poi_.x, poi_.y,
  // poi_.z}};
  std::array<double, 4> initial_pose{{poi_.x, poi_.y, search_area_height_}};
  // float height_step = search_area_height_ / (seek_points.size() - 1);
  float yaw_step = M_PI_4;
  float yaw = 0.0;
  float max_yaw = 1.9 * M_PI;

  for (int i = 0; i < seek_points.size(); i++) {
    seek_points[i][0] = poi_.x;
    seek_points[i][1] = poi_.y;
    // seek_points[i][2] = initial_pose[2] - i*height_step;
    // seek_points[i][2] = poi_.z;
    seek_points[i][2] = search_area_height_;
    yaw = yaw + yaw_step;
    if (yaw > max_yaw) {
      yaw = 0.0;
    }
    seek_points[i][3] = yaw;

    // std::cout << seek_points[i][0] << " " << seek_points[i][1] << " " <<
    // seek_points[i][2] << " " << seek_points[i][3] << std::endl;
  }

  for (int i = 0; i < seek_points.size(); i++) {
    poses_[i].position.x = seek_points[i][0];
    poses_[i].position.y = seek_points[i][1];
    poses_[i].position.z = seek_points[i][2];
    // poses_[i].orientation.x = seek_points[i][3];
    // poses_[i].orientation.y = seek_points[i][4];
    // poses_[i].orientation.z = seek_points[i][5];
    poses_[i].orientation.w = seek_points[i][3]; // YAW
  }

  run_node_ = true;
  waypoint_sent_ = false;

  runDetector(true);
}

void GoalSeeker::runDetector(bool _run_node) {
  std_msgs::Bool msg;
  msg.data = _run_node;
  run_detection_pub_.publish(msg);
}

void GoalSeeker::stop() {
  run_node_ = false;
  ROS_INFO("Node stopped");
}

void GoalSeeker::endSearch() {
  std_msgs::Bool msg;
  msg.data = target_found_;
  has_ended_pub_.publish(msg);
  // run_node_ = false;
  ROS_INFO("End search");
  runDetector(false);
}

void GoalSeeker::run() {
  if (!run_node_) {
    runDetector(false);
    return;
  }

  if (!odometry_received_) {
    ROS_INFO_ONCE("Waiting for odometry...");
    return;
  }

  runDetector(true);

  // SEEK
  // if (tag_pose_received_)
  // {
  //   return;
  // }

  // Find near walls
  if (find_nearest_wall_) {
    // LIST OF YAW VISITED?
    std::vector<float> yaws;
    if (findYawsOfInterest(yaws)) {
      for (float yaw : yaws) {
        std::cout << "Yoi: " << yaw << std::endl;
        // Insert yaws that are neither
        // in yaw_nearest_wall nor yaw_visited
        near_walls_yaw_.insert(yaw);
      }
    }
  }

  if (waypoint_sent_) {
    // double distance = sqrt(pow(poses_[order_index_].position.x -
    // odometry_.pose.pose.position.x, 2) + pow(poses_[order_index_].position.y
    // - odometry_.pose.pose.position.y, 2));
    double distance = sqrt(
        pow(poses_[order_index_].position.x - odometry_.pose.pose.position.x,
            2) +
        pow(poses_[order_index_].position.y - odometry_.pose.pose.position.y,
            2) +
        pow(poses_[order_index_].position.z - odometry_.pose.pose.position.z,
            2));
    auto msg = generateWaypointMsg(poses_[order_index_], ref_angle_);
    waypoint_pub_.publish(msg);
    // std::cout << "Distance: " << distance << std::endl;
    if (distance < next_point_reached_dist_) {
      // std::cout << "Distance reached" << std::endl;
      if (checkOrientationReached(ref_angle_)) {
        waypoint_sent_ = false;
        order_index_ += modifier_;
        // Continue searching backwards
        // if (order_index_ == (poses_.size() - 1) || order_index_ == 0)
        // {
        //   modifier_ *= -1;
        // }
        // Finish searching backwards
        // if (order_index_ == 0) {
        // Finish searching forwards
        // if (order_index_ == (poses_.size() - 1)) {
        if (order_index_ == (poses_.size() - 1) || order_index_ == 0) {
          endSearch();
        }
      }
    }
  } else {
    ref_angle_ = poses_[order_index_].orientation.w;
    // NEAREST POINT APPROACH (ONE YAW)
    // if (find_nearest_wall_) {
    // float yaw = 0.0;
    // if (findYawOfInterest(yaw)){
    //   ref_angle_ = yaw;
    // }
    // }

    // INSPECT SEVERAL YAWS
    // ref_angle_ = next yaw not visited
    // check if yaw_visited_
  }
  // ROS_INFO("Sending waypoint %f, %f, %f. Angle %f",
  // poses_[order_index_].position.x, poses_[order_index_].position.y,
  // poses_[order_index_].position.z, ref_angle_);
  auto msg = generateWaypointMsg(poses_[order_index_], ref_angle_);
  waypoint_pub_.publish(msg);
  // pose_pub_.publish(generatePoseStampedMsg(poses_[order_index_],
  // ref_angle_));
  waypoint_sent_ = true;
}

// Check final condition and send has_ended

// INSPECTION
// if (tag_pose_received_)
// {
//   geometry_msgs::Pose seek_pose_;
//   Eigen::Vector3d seek_vector = identifyTagOrientation(tag_position_);
//   float inspection_distance = inspection_distance_; // in meters
//   seek_pose_.position.x = tag_position_.x() + seek_vector.x() *
//   inspection_distance; seek_pose_.position.y = tag_position_.y() +
//   seek_vector.y() * inspection_distance; seek_pose_.position.z =
//   tag_position_.z() + inspection_height_;

//   ref_angle_ = identifySeekYaw(tag_position_);

//   // ROS_INFO("INSPECTION POINT SENDED");

//   // waypoint_pub_.publish(generateWaypointMsg(seek_pose_, ref_angle_));
//   pose_pub_.publish(generatePoseStampedMsg(seek_pose_, ref_angle_));
// }
// }

bool GoalSeeker::checkOrientationReached(const float _angle) {
  tf2::Quaternion q(
      odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y,
      odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  if (yaw < 0) {
    yaw = 2 * M_PI + yaw;
  }
  if (yaw > 1.9 * M_PI) {
    yaw = yaw - 2 * M_PI;
  }

  // std::cout << "RefAngle: " << _angle << " Yaw: " << yaw << std::endl;

  if (fabs(_angle - yaw) < next_point_reached_yaw_) {
    return true;
  }
  return false;
}

bool GoalSeeker::controlNodeSrv(std_srvs::SetBool::Request &_request,
                                std_srvs::SetBool::Response &_response) {
  if (run_node_ == _request.data) {
    ROS_INFO("Control Node: ALREADY RUNNING");
    _response.success = false;
    _response.message = "Run node already set to " + std::to_string(run_node_);
    return true;
  }

  _response.success = true;
  _response.message = "Run node set to " + std::to_string(run_node_);

  if (_request.data) {
    ROS_INFO("Control Node: RUN");
    start();
  } else {
    ROS_INFO("Control Node: STOP");
    stop();
  }

  return true;
}

bool GoalSeeker::findYawsOfInterest(std::vector<float> &_yoi)
// bool GoalSeeker::findYawOfInterest(float &_yoi)
{
  // return false;
  cv::Point2i center_cell(0, 0);
  cv::Size map_size;
  int max_radious = 0;
  try {
    map_size = map_.size();
    max_radious = std::min(map_size.height, map_size.width);
    center_cell.x = map_size.height / 2;
    center_cell.y = map_size.width / 2;
  } catch (...) {
    ROS_WARN("Map not received");
  }

  if (max_radious < 1) {
    return false;
  }

  int n_sectors = 16;
  double sector_angle = 360.0 / n_sectors;

  cv::Mat sectors = cv::Mat::zeros(map_size, map_.type());

  for (int i = 0; i < n_sectors; i++) {
    double init_angle = i * sector_angle;
    double end_angle = (i + 1) * sector_angle;
    double mid_yaw_angle = (i + 0.5) * sector_angle * PI / 180;

    cv::Mat mask = cv::Mat::zeros(map_size, CV_8UC1);
    cv::ellipse(mask, cv::Point(center_cell.x, center_cell.y),
                cv::Size(max_radious, max_radious), 0, init_angle, end_angle,
                cv::Scalar(255), -1);
    cv::Mat sector = map_ & mask;
    int free_pixels = cv::countNonZero(mask);
    int occupied_pixels = free_pixels - cv::countNonZero(sector);

    std::cout << "Occ px: " << occupied_pixels << std::endl;

    int occ_th = 5;
    if (occupied_pixels > occ_th) {
      // std::cout << "Yaw " << mid_angle << std::endl;
      _yoi.emplace_back(mid_yaw_angle);
    }

    // DEBUG
    // std::cout << "Image size: " << sector.size() << std::endl;
    // std::cout << "Sector " << i + 1 << ": " << occupied_pixels << std::endl;
    // std::cout << "Yaw " << mid_yaw_angle << std::endl;
    // cv::namedWindow("Map", cv::WINDOW_FREERATIO);
    // cv::imshow("Map", map_);
    // cv::namedWindow("Mask", cv::WINDOW_FREERATIO);
    // cv::imshow("Mask", mask);
    // cv::namedWindow("Sector", cv::WINDOW_FREERATIO);
    // cv::imshow("Sector", sector);
    // cv::waitKey(0);
  }

  if (_yoi.size() > 0) {
    std::cout << "Size: " << _yoi.size() << std::endl;
    return true;
  }
  return false;
}

bool GoalSeeker::findYawOfInterest(float &_yoi) {
  // return false;
  cv::Point2i center_cell(0, 0);
  int max_radious = 0;
  try {
    cv::Size map_size = map_.size();
    max_radious = std::min(map_size.height, map_size.width);
    center_cell.x = map_size.height / 2;
    center_cell.y = map_size.width / 2;
  } catch (...) {
    ROS_WARN("Map not received");
  }

  if (max_radious < 1) {
    return false;
  }

  // if map empty (or almost empty)
  // return None

  cv::Point2i nearest_cell = center_cell;
  cv::Point2i cell = center_cell;
  bool cell_found = false;

  // FIX: MAKE CIRCLES
  for (int radious = 1; radious < max_radious; radious++) {
    int min_distance = max_radious * 2;
    for (int i = -radious; i <= radious; i++) {
      for (int j = -radious; j <= radious; j++) {
        cell.x = center_cell.x + i;
        cell.y = center_cell.y + j;
        if (cellIsOccupied(cell)) {
          int distance = abs(i) + abs(j);
          if (distance < min_distance) {
            min_distance = distance;
            nearest_cell = cell;
            cell_found = true;
          }
        }
      }
    }
    if (cell_found) {
      _yoi = calculateYaw(nearest_cell, center_cell);
      return true;
    }
  }
  return false;
}

float GoalSeeker::calculateYaw(cv::Point2i &_cell, cv::Point2i &_center) {
  cv::Size map_size = map_.size();
  cv::Point2i cell_0(map_size.height / 2, map_size.width);

  float yaw_i = atan2(_cell.y - _center.y, _cell.x - _center.x);
  float yaw_0 = atan2(cell_0.y - _center.y, cell_0.x - _center.x);
  float yaw = (atan2(_cell.y - _center.y, _cell.x - _center.x) -
               atan2(cell_0.y - _center.y, cell_0.x - _center.x));

  if (yaw < 0) {
    yaw = 2 * M_PI + yaw;
  }
  return yaw;
}

bool GoalSeeker::cellIsOccupied(const cv::Point2i &_cell) {
  // if (cell==0)
  //   Check is alone
  return map_.at<uchar>(_cell.x, _cell.y) == 0;
}

// CALLBACKS
void GoalSeeker::odometryCallback(const nav_msgs::Odometry::ConstPtr &_msg) {
  odometry_received_ = true;
  odometry_ = *_msg;
}

void GoalSeeker::mapCallback(const sensor_msgs::ImageConstPtr &_map) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(_map, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat original_map = cv_ptr->image;
    cv::Mat filtered_map, eroded_map;
    cv::Mat element =
        cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::dilate(original_map, eroded_map, element);
    cv::erode(eroded_map, map_, element);

    // cv::namedWindow("filtered", cv::WINDOW_FREERATIO);
    // cv::imshow("filtered", filtered_map);
    // cv::waitKey(1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

// SERVERS
bool GoalSeeker::setGoalSrv(path_planner::setGoalPoint::Request &_request,
                            path_planner::setGoalPoint::Response &_response) {

  if (run_node_) {
    _response.success = false;
    _response.message = "Node already running";
    ROS_WARN("Node already running");
    return false;
  }

  // Check initial distance
  // float distance =

  poi_ = _request.goal.point;

  ROS_INFO("POI: %f, %f, %f", poi_.x, poi_.y, poi_.z);

  _response.success = true;
  _response.message = "POI set to: " + std::to_string(poi_.x) + ", " +
                      std::to_string(poi_.y) + ", " + std::to_string(poi_.z);

  start();

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////

trajectory_msgs::MultiDOFJointTrajectoryPoint
generateWaypointMsg(const geometry_msgs::Pose &_poses, const float _yaw) {
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

geometry_msgs::PoseStamped
generateGoalPoseMsg(const Eigen::Vector3d _goal_position) {
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

geometry_msgs::PoseStamped
generatePoseStampedMsg(const geometry_msgs::Pose _waypoint, const float _yaw) {
  // convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, _yaw);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x = _waypoint.position.x;
  pose_msg.pose.position.y = _waypoint.position.y;
  pose_msg.pose.position.z = _waypoint.position.z;
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();
  return pose_msg;
}
