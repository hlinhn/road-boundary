#include <enway_helper/param_helper.hpp>
#include <ios>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/types.hpp>
#include <road_boundary/bag_process.h>
#include <ros/node_handle.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <string>
#include <vector>

road_boundary::BagProcess::BagProcess(ros::NodeHandle& node_handle)
{
  const auto path = enway_helper::getParam<std::string>(node_handle, "path");
  road_boundary_.setPath(path);
  const auto distance_threshold = enway_helper::getParam<double>(node_handle, "distance_threshold");
  road_boundary_.params_.cosine_threshold = enway_helper::getParam<double>(node_handle, "cosine_threshold");
  road_boundary_.params_.max_distance_search = enway_helper::getParam<double>(node_handle, "search_threshold");
  road_boundary_.params_.split_distance_threshold =
      enway_helper::getParam<double>(node_handle, "split_distance_threshold");
  road_boundary_.params_.min_set_size = enway_helper::getParam<int>(node_handle, "min_set_size");
  road_boundary_.params_.tolerance = enway_helper::getParam<double>(node_handle, "tolerance");

  const auto bag_path = enway_helper::getParam<std::string>(node_handle, "bag");
  const auto topic_name = enway_helper::getParam<std::string>(node_handle, "topic");
  std::vector<std::string> topic;
  topic.push_back(topic_name);

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::BagMode::Read);

  bool first_point = false;
  cv::Point2i last_point;
  ros::Time last_time;

  std::vector<cv::Point2i> left_points;
  std::vector<cv::Point2i> right_points;
  for (rosbag::MessageInstance const message : rosbag::View(bag, rosbag::TopicQuery(topic)))
  {
    const auto position = message.instantiate<nav_msgs::Odometry>();
    const auto index = road_boundary_.convertToIndex(position->pose.pose.position.x, position->pose.pose.position.y);
    if (!first_point)
    {
      last_point = index;
      first_point = true;
      last_time = position->header.stamp;
    }
    if ((position->header.stamp - last_time).toSec() < 1.5)
    {
      continue;
    }
    if (std::abs(index.x - last_point.x) + std::abs(index.y - last_point.y) < distance_threshold)
    {
      continue;
    }
    const auto index_left_right = road_boundary_.findSides(index, last_point);
    if (index_left_right.first)
    {
      left_points.push_back(index_left_right.first.value());
    }
    if (index_left_right.second)
    {
      right_points.push_back(index_left_right.second.value());
    }
    last_point = index;
    last_time = position->header.stamp;
  }
  road_boundary_.fitCurve(left_points);
  road_boundary_.fitCurve(right_points);

  // for (auto p : left_points)
  // {
  //   if (road_boundary_.point_to_func_.count(p) > 0)
  //   {
  //     if (road_boundary_.left_to_right_.count(p) > 0)
  //     {
  //       auto corres = road_boundary_.left_to_right_[p];
  //       if (road_boundary_.point_to_func_.count(corres) > 0)
  //       {
  //         road_boundary_.debugDraw(p, false);
  //         road_boundary_.debugDraw(corres, false);
  //       }
  //     }
  //     else
  //     {
  //       road_boundary_.debugDraw(p, true);
  //     }
  //   }
  // }
  // for (auto p : right_points)
  // {
  //   if (road_boundary_.point_to_func_.count(p) > 0)
  //   {
  //     if (road_boundary_.right_to_left_.count(p) == 0)
  //     {
  //       road_boundary_.debugDraw(p, true);
  //     }
  //   }
  // }

  auto split_line = road_boundary_.splitLine(left_points, right_points);
  std::vector<std::vector<cv::Point2d>> fin_data;
  for (auto line : split_line)
  {
    auto gps = road_boundary_.convertToGPS(line);
    fin_data.push_back(gps);
  }

  road_boundary_.writeToOsmFile(fin_data);
  road_boundary_.saveImage();
  bag.close();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_process");
  ros::NodeHandle node_handle("~");
  road_boundary::BagProcess process(node_handle);

  return 0;
}
