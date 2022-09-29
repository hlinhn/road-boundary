/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef ROAD_BOUNDARY_NODE_H_
#define ROAD_BOUNDARY_NODE_H_

#include <opencv2/core/types.hpp>
#include <ros/service_server.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma GCC diagnostic ignored "-Wsuggest-override"
#include "road_boundary/RoadBoundaryConfig.h"
#pragma GCC diagnostic pop

#include "road_boundary/road_boundary.h"

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <std_srvs/Empty.h>

namespace road_boundary
{
class Node
{
public:
  Node(ros::NodeHandle& node_handle);

private:
  ros::Subscriber position_sub_;
  ros::Subscriber cartographer_sub_;
  ros::ServiceServer save_image_srv_;
  ros::ServiceServer save_node_srv_;
  ros::Publisher distance_pub_;

  ros::Time last_saved_;
  double distance_threshold_;
  double time_threshold_;
  bool first_point_;
  cv::Point2i last_point_;

  bool saveImageCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool saveNodeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void positionCallback(const nav_msgs::Odometry& position);
  void cartographerCallback(const geometry_msgs::PoseWithCovarianceStamped& position);
  void findLeftAndRightEdge(cv::Point2i index);
  void reconfigureCallback(RoadBoundaryConfig& config, uint32_t /* level */);
  void writeToOSMFile(const std::vector<cv::Point2d> nodes);

  std::vector<cv::Point2d> convertToGPS(const std::vector<cv::Point2i> points);
  RoadBoundary road_boundary_;
  dynamic_reconfigure::Server<RoadBoundaryConfig> dyn_reconf_server_;

  std::vector<cv::Point2i> left_points_;
  std::vector<cv::Point2i> right_points_;
};

} // namespace road_boundary

#endif
