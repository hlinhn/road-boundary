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

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_matching/Types.h>
#include <lanelet2_projection/UTM.h>

#include <road_boundary/helper.h>
#include <road_boundary/road_graph.h>

road_boundary::BagProcess::BagProcess(RoadBoundaryConfig config)
{
  road_boundary_.setConfig(config);
  road_boundary_.setPath(config.image_path);
  RouteGraph graph(config.route_graph);

  std::vector<std::string> topic;
  topic.push_back(config.topic_name);

  rosbag::Bag bag;
  bag.open(config.bag_path, rosbag::BagMode::Read);

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
    if ((position->header.stamp - last_time).toSec() < config.min_time_diff)
    {
      continue;
    }
    if (distance(index, last_point) < config.distance_threshold / config.resolution)
    {
      continue;
    }
    const auto index_left_right = road_boundary_.findSides(index, last_point);
    const auto last_point_index = index_left_right.size() - 1;
    if (index_left_right[0])
    {
      left_points.push_back(index_left_right[0].value());
    }
    if (index_left_right[last_point_index])
    {
      right_points.push_back(index_left_right[last_point_index].value());
    }
    if (index_left_right[0] && index_left_right[last_point_index])
    {
      auto chosen_point = index_left_right[0].value();
      for (int i = 1; i < index_left_right.size() - 1; i++)
      {
        if (distance(chosen_point, index_left_right[i].value()) < 3.0 / config.resolution
            || distance(index_left_right[last_point_index].value(), index_left_right[i].value())
                   < 3.0 / config.resolution)
        {
          continue;
        }
        chosen_point = index_left_right[i].value();
        // road_boundary_.debugDraw(index_left_right[i].value(), false);
      }
    }
    last_point = index;
    last_time = position->header.stamp;
  }

  std::vector<cv::Point2i> left_filtered;
  std::vector<cv::Point2i> right_filtered;

  for (auto point : left_points)
  {
    if (graph.nearIntersection(point))
    {
      road_boundary_.debugDraw(point, true);
      continue;
    }
    left_filtered.push_back(point);
  }
  for (auto point : right_points)
  {
    if (graph.nearIntersection(point))
    {
      road_boundary_.debugDraw(point, true);
      continue;
    }
    right_filtered.push_back(point);
  }

  road_boundary_.fitCurve(left_filtered);
  road_boundary_.fitCurve(right_filtered);

  auto split_line = road_boundary_.splitLine(left_filtered, right_filtered);
  std::vector<std::vector<cv::Point2d>> fin_data;

  for (auto line : split_line)
  {
    auto gps = road_boundary_.convertToGPS(line);
    fin_data.push_back(gps);
  }

  road_boundary_.writeToOsmFile(fin_data);

  lanelet::ErrorMessages errors;
  lanelet::Origin origin({1.3541351067 - 0.0015, 103.695233561 + 0.00491});
  auto projector = lanelet::projection::UtmProjector(origin);
  // lanelet::Origin origin({49.0, 8.4});
  lanelet::io_handlers::OsmParser parser(projector);
  // lanelet::LaneletMapPtr map = lanelet::load("/home/linh/.ros/test_osm_nodes.osm", origin, &errors);
  auto map = parser.parse("/home/linh/.ros/test_osm_nodes.osm", errors);
  double angle = 62;
  double cos = std::cos(angle * M_PI / 180.0);
  double sin = std::sin(angle * M_PI / 180.0);

  first_point = false;
  int count = 0;
  std::cout << "=========================" << std::endl;
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
    if ((position->header.stamp - last_time).toSec() < 2.5)
    {
      continue;
    }
    if (std::abs(index.x - last_point.x) + std::abs(index.y - last_point.y) < 500)
    {
      continue;
    }
    auto gpspoint = projector.reverse(
        lanelet::BasicPoint3d(-(index.y * cos + index.x * sin) * 0.1, -(index.x * cos - index.y * sin) * 0.1, 0));
    std::cout << -(index.y * cos + index.x * sin) * 0.1 << " " << -(index.x * cos - index.y * sin) * 0.1 << " vs ";
    auto normal_point = projector.forward(gpspoint);
    std::cout << normal_point.x() << " " << normal_point.y() << std::endl;
    lanelet::matching::Object2d query;
    // query.pose.translation() = lanelet::BasicPoint2d {normal_point.x(), normal_point.y()};
    query.pose.translation() =
        lanelet::BasicPoint2d {-(index.y * cos + index.x * sin) * 0.1, -(index.x * cos - index.y * sin) * 0.1};
    query.pose.linear() = Eigen::Rotation2D<double>(0.0).matrix();

    auto matching = lanelet::matching::getDeterministicMatches(*map, query, 50.0);
    if (matching.size() > 0)
    {
      road_boundary_.drawId(index, matching[0].lanelet.id());
    }
    else
    {
      std::cout << "No match\n";
    }
    last_point = index;
    last_time = position->header.stamp;
    count++;
  }

  std::cout << "Processed " << count << std::endl;
  road_boundary_.saveImage();
  // road_boundary_.testROI(split_line);
  bag.close();
}

int
main(int argc, char** argv)
{
  auto config = readConfig(std::string(argv[1]));
  road_boundary::BagProcess processor(config);
  return 0;
}
