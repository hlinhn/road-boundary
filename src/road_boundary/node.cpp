/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "road_boundary/node.h"

#include "road_boundary/polyfit.hpp"

#include <cstddef>
#include <ctime>
#include <enway_helper/param_helper.hpp>
#include <opencv2/core/types.hpp>
#include <ostream>
#include <std_msgs/Float64.h>

road_boundary::Node::Node(ros::NodeHandle& node_handle)
  : road_boundary_ {}
  , first_point_ {true}
  , last_saved_ {ros::Time(0)}
  , dyn_reconf_server_ {node_handle}
{
  distance_threshold_ = enway_helper::getParam<double>(node_handle, "distance_threshold");
  time_threshold_ = enway_helper::getParam<double>(node_handle, "time_threshold");

  const auto path = enway_helper::getParam<std::string>(node_handle, "path");
  road_boundary_.setPath(path);
  road_boundary_.params_.cosine_threshold = enway_helper::getParam<double>(node_handle, "cosine_threshold");
  road_boundary_.params_.max_distance_search = enway_helper::getParam<double>(node_handle, "search_threshold");
  road_boundary_.params_.split_distance_threshold =
      enway_helper::getParam<double>(node_handle, "split_distance_threshold");
  road_boundary_.params_.min_set_size = enway_helper::getParam<int>(node_handle, "min_set_size");
  road_boundary_.params_.tolerance = enway_helper::getParam<double>(node_handle, "tolerance");

  position_sub_ = node_handle.subscribe("/global_odom", 1, &Node::positionCallback, this);
  // cartographer_sub_ =
  //     node_handle.subscribe("/cartographer/tracked_pose_with_covariance", 1, &Node::cartographerCallback, this);
  distance_pub_ = node_handle.advertise<std_msgs::Float64>("distance", 1, true);
  save_image_srv_ = node_handle.advertiseService("save_image", &Node::saveImageCallback, this);
  save_node_srv_ = node_handle.advertiseService("save_node", &Node::saveNodeCallback, this);
  dyn_reconf_server_.setCallback([this](auto&& config, auto&& level) { reconfigureCallback(config, level); });
}

bool
road_boundary::Node::saveNodeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  // auto all_nodes = convertToGPS(left_points_);
  // const auto right_nodes = convertToGPS(right_points_);
  // all_nodes.insert(all_nodes.end(), right_nodes.begin(), right_nodes.end());
  // ROS_INFO_STREAM("Number of nodes " << all_nodes.size());
  // writeToOSMFile(all_nodes);
  return true;
}

bool
road_boundary::Node::saveImageCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  road_boundary_.fitCurve(left_points_);
  road_boundary_.fitCurve(right_points_);
  road_boundary_.saveImage();
  return true;
}

void
road_boundary::Node::findLeftAndRightEdge(cv::Point2i index)
{
  const auto index_left_right = road_boundary_.findSides(index, last_point_);
  if (index_left_right.first)
  {
    road_boundary_.debugDraw(index_left_right.first.value(), false);
    left_points_.push_back(index_left_right.first.value());
  }
  if (index_left_right.second)
  {
    road_boundary_.debugDraw(index_left_right.second.value(), false);
    right_points_.push_back(index_left_right.second.value());
  }
  if (index_left_right.first && index_left_right.second)
  {
    const auto distance = road_boundary::pointDistance(index_left_right.first.value(), index_left_right.second.value());
    std_msgs::Float64 msg;
    msg.data = distance;
    distance_pub_.publish(msg);
  }
  // road_boundary_.debugDraw(index, true);
}

void
road_boundary::Node::positionCallback(const nav_msgs::Odometry& position)
{
  if ((position.header.stamp - last_saved_).toSec() < time_threshold_)
  {
    return;
  }
  last_saved_ = position.header.stamp;

  const auto index = road_boundary_.convertToIndex(position.pose.pose.position.x, position.pose.pose.position.y);

  if (std::abs(index.x - last_point_.x) + std::abs(index.y - last_point_.y) < distance_threshold_)
  {
    return;
  }
  if (first_point_)
  {
    last_point_ = index;
    first_point_ = false;
    return;
  }
  findLeftAndRightEdge(index);
  last_point_ = index;
}

void
road_boundary::Node::cartographerCallback(const geometry_msgs::PoseWithCovarianceStamped& position)
{
  if ((position.header.stamp - last_saved_).toSec() < 5.0)
  {
    return;
  }
  last_saved_ = position.header.stamp;

  const auto index = road_boundary_.convertToIndex(position.pose.pose.position.x, position.pose.pose.position.y);
  if (std::abs(index.x - last_point_.x) + std::abs(index.y - last_point_.y) < 5)
  {
    return;
  }

  if (first_point_)
  {
    last_point_ = index;
    first_point_ = false;
    return;
  }
  findLeftAndRightEdge(index);
  last_point_ = index;
}

void
road_boundary::Node::reconfigureCallback(RoadBoundaryConfig& config, uint32_t /* level */)
{
  config.__clamp__(); // dummy to not get unused compile error
}
