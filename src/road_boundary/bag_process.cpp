#include <ios>
#include <opencv2/core/types.hpp>
#include <road_boundary/bag_process.h>
#include <string>
#include <vector>

#include <road_boundary/helper.h>
#include <road_boundary/road_graph.h>

road_boundary::BagProcess::BagProcess(RoadBoundaryConfig config)
{
  road_boundary_.setConfig(config);

  bool first_point = false;
  cv::Point2i last_point;
  ros::Time last_time;

  std::vector<cv::Point2i> left_points;
  std::vector<cv::Point2i> right_points;
  auto points = parseBagPath(config.bag_path,
                             cv::Size(road_boundary_.image_size_, road_boundary_.image_size_),
                             config.resolution,
                             config.topic_name);
  for (auto index : points)
  {
    if (!first_point)
    {
      last_point = index;
      first_point = true;
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
        if (distance(chosen_point, index_left_right[i].value()) < config.lane_min_width / config.resolution
            || distance(index_left_right[last_point_index].value(), index_left_right[i].value())
                   < config.lane_min_width / config.resolution)
        {
          continue;
        }
        chosen_point = index_left_right[i].value();
        // road_boundary_.debugDraw(index_left_right[i].value(), false);
      }
    }
    last_point = index;
  }

  std::vector<cv::Point2i> left_filtered;
  std::vector<cv::Point2i> right_filtered;

  if (config.route_graph.empty())
  {
    left_filtered = left_points;
    right_filtered = right_points;
  }
  else
  {
    RouteGraph graph(config.route_graph);

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

  road_boundary_.saveImage();
}

int
main(int argc, char** argv)
{
  auto config = readConfig(std::string(argv[1]));
  road_boundary::BagProcess processor(config);
  return 0;
}
