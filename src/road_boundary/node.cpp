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
#include <osmium/builder/attr.hpp>
#include <osmium/io/any_output.hpp>

#include <osmium/osm/location.hpp>
#include <osmium/builder/osm_object_builder.hpp>

#include <std_msgs/Float64.h>

road_boundary::Node::Node(ros::NodeHandle& node_handle)
  : road_boundary_ {}
  , first_point_ {true}
  , last_saved_ {ros::Time(0)}
  , dyn_reconf_server_ {node_handle}
{
  const auto path = enway_helper::getParam<std::string>(node_handle, "path");
  road_boundary_.setPath(path);
  // position_sub_ = node_handle.subscribe("/global_odom", 1, &Node::positionCallback, this);
  cartographer_sub_ =
      node_handle.subscribe("/cartographer/tracked_pose_with_covariance", 1, &Node::cartographerCallback, this);
  distance_pub_ = node_handle.advertise<std_msgs::Float64>("distance", 1, true);
  save_image_srv_ = node_handle.advertiseService("save_image", &Node::saveImageCallback, this);
  save_node_srv_ = node_handle.advertiseService("save_node", &Node::saveNodeCallback, this);
  dyn_reconf_server_.setCallback([this](auto&& config, auto&& level) { reconfigureCallback(config, level); });
}

void
road_boundary::Node::writeToOSMFile(const std::vector<cv::Point2d> nodes)
{
  const osmium::io::File output_file {"test_osm_nodes", "xml"};
  osmium::io::Header header;
  header.set("generator", "test_nodes");

  const size_t initial_buffer_size = 10000;
  osmium::memory::Buffer buffer {initial_buffer_size, osmium::memory::Buffer::auto_grow::yes};
  using namespace osmium::builder::attr;
  for (size_t i = 0; i < nodes.size(); i++)
  {
    osmium::builder::add_node(buffer,
                              _id(i),
                              _version(1),
                              _timestamp(std::time(nullptr)),
                              _location(osmium::Location {nodes[i].x, nodes[i].y}));
  }
  osmium::io::Writer writer {output_file, header, osmium::io::overwrite::allow};
  writer(std::move(buffer));
  writer.close();
}

std::vector<cv::Point2d>
road_boundary::Node::convertToGPS(const std::vector<cv::Point2i> points)
{
  std::vector<cv::Point2d> gps_points;

  for (const auto point : points)
  {
    cv::Point2d gps;
    gps.x = static_cast<double>(point.x);
    gps.y = static_cast<double>(point.y);
    gps_points.push_back(gps);
  }
  return gps_points;
}

bool
road_boundary::Node::saveNodeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  auto all_nodes = convertToGPS(left_points_);
  const auto right_nodes = convertToGPS(right_points_);
  all_nodes.insert(all_nodes.end(), right_nodes.begin(), right_nodes.end());

  writeToOSMFile(all_nodes);
  return true;
}

bool
road_boundary::Node::saveImageCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  road_boundary_.saveImage();
  return true;
}

double
pointDistance(cv::Point2i p, cv::Point2i q)
{
  const auto dx = p.x - q.x;
  const auto dy = p.y - q.y;

  return std::sqrt(dx * dx + dy * dy);
}

void
road_boundary::Node::fitCurve(std::vector<cv::Point2i> points)
{
  const auto size = points.size();
  if (size < 10)
  {
    return;
  }
  if (pointDistance(points[0], points[size - 1]) < 5.0)
  {
    return;
  }
  std::vector<cv::Point2i> output;

  std::vector<double> xs;
  std::vector<double> ys;

  for (int i = size - 10; i < size; i++)
  {
    xs.push_back(points[i].x);
    ys.push_back(points[i].y);
  }
  std::vector<double> results = polyfit_Eigen(xs, ys, 1);
  const auto fitted_val = polyval(results, xs);

  for (unsigned int i = 0; i < xs.size(); i++)
  {
    std::cout << ys[i] << " " << fitted_val[i] << std::endl;
    cv::Point2i p;
    p.x = static_cast<int>(xs[i]);
    p.y = static_cast<int>(fitted_val[i]);
    output.push_back(p);
  }
  std::cout << "=============\n";
  road_boundary_.drawApproximateLine(output);
}

void
road_boundary::Node::findLeftAndRightEdge(cv::Point2i index)
{
  const auto index_left_right = road_boundary_.findSides(index, last_point_);
  if (index_left_right.first)
  {
    road_boundary_.debugDraw(index_left_right.first.value(), false);
    left_points_.push_back(index_left_right.first.value());
    fitCurve(left_points_);
  }
  if (index_left_right.second)
  {
    road_boundary_.debugDraw(index_left_right.second.value(), false);
    right_points_.push_back(index_left_right.second.value());
  }
  if (index_left_right.first && index_left_right.second)
  {
    const auto distance = pointDistance(index_left_right.first.value(), index_left_right.second.value());
    std_msgs::Float64 msg;
    msg.data = distance;
    distance_pub_.publish(msg);
  }
  road_boundary_.debugDraw(index, true);
}

void
road_boundary::Node::positionCallback(const nav_msgs::Odometry& position)
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
  // findLeftAndRightEdge(index);
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
  // findLeftAndRightEdge(index);
  last_point_ = index;
}

void
road_boundary::Node::reconfigureCallback(RoadBoundaryConfig& config, uint32_t /* level */)
{
  config.__clamp__(); // dummy to not get unused compile error
}
