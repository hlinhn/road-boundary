/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef ROAD_BOUNDARY_ROAD_BOUNDARY_H_
#define ROAD_BOUNDARY_ROAD_BOUNDARY_H_

#include <bits/stdint-uintn.h>
#include <map>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <string>

namespace road_boundary
{
double pointDistance(cv::Point2i p, cv::Point2i q);
struct ComparePoints
{
  bool
  operator()(cv::Point2i const& a, cv::Point2i const& b) const
  {
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
  }
};

struct BoundaryParam
{
  double resolution;
  double max_distance_search;
  double split_distance_threshold;
  double cosine_threshold;
  int min_set_size;
  double tolerance;
};

class RoadBoundary
{
public:
  RoadBoundary();

  BoundaryParam params_;
  void setPath(const std::string& path);
  cv::Point2i convertToIndex(const double x, const double y);
  std::pair<std::optional<cv::Point2i>, std::optional<cv::Point2i>> findSides(const cv::Point2i current,
                                                                              const cv::Point2i last_point);
  void debugDraw(cv::Point2i point, bool root);
  void saveImage();
  void drawApproximateLine(std::vector<cv::Point2i> points, int func_id);
  void drawCubicBezier(std::vector<cv::Point2d> controls);
  void fitCurve(std::vector<cv::Point2i> points);
  void writeToOsmFile(const std::vector<std::vector<cv::Point2d>> points);
  std::vector<std::vector<cv::Point2i>> splitLine(std::vector<cv::Point2i> left_points,
                                                  std::vector<cv::Point2i> right_points);
  std::vector<cv::Point2d> convertToGPS(const std::vector<cv::Point2i> points);

private:
  int func_id_counter_;
  cv::Mat image_;
  cv::Mat debug_image_;
  std::size_t image_size_;
  std::map<cv::Point2i, cv::Point2i, road_boundary::ComparePoints> left_to_right_;
  std::map<cv::Point2i, cv::Point2i, road_boundary::ComparePoints> right_to_left_;
  std::map<cv::Point2i, int, road_boundary::ComparePoints> point_to_func_;
  std::map<int, std::vector<double>> func_id_map_;
  std::map<int, int> lanelet_id_map_;

  bool read(const std::string& file_path);
  cv::Point2d getDirection(const cv::Point2i prev, const cv::Point2i cur);
  std::optional<cv::Point2i> searchPoint(const cv::Point2i current, const cv::Point2d direction);
  std::optional<cv::Vec<uint16_t, 4>> queryPoint(const cv::Point2i index);
  bool checkPoint(const cv::Point2i index);
  std::vector<std::vector<cv::Point2i>> splitPoints(std::vector<cv::Point2i> points);
};

} // namespace road_boundary

#endif
