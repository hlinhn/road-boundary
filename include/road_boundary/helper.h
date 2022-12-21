#ifndef ROAD_BOUNDARY_HELPER_H_
#define ROAD_BOUNDARY_HELPER_H_

#include <opencv2/core/types.hpp>
#include <string>

struct RoadBoundaryConfig
{
  std::string image_path;
  std::string route_graph;
  std::string bag_path;
  double cosine_threshold;
  double distance_threshold;
  double search_threshold;
  double split_distance_threshold;
  int min_set_size;
  double tolerance;
  std::string topic_name;
  double resolution;
  double lane_min_width;
  double min_time_diff;
  int min_curve_size;
  double min_distance_curve;
  std::string save_folder;
  unsigned int curb_threshold;
};

RoadBoundaryConfig readConfig(std::string filename);
double distance(cv::Point2i p, cv::Point2i q);
cv::Point2i convertToIndex(const double x, const double y, const double resolution, const cv::Size image_size);
std::vector<cv::Point2i> parseBagPath(std::string bag_path, cv::Size image_size, double resolution, std::string topic);

#endif
