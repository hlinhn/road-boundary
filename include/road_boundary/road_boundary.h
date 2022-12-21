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

#include <road_boundary/helper.h>

namespace road_boundary
{
struct ComparePoints
{
  bool
  operator()(cv::Point2i const& a, cv::Point2i const& b) const
  {
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
  }
};

class RoadBoundary
{
public:
  RoadBoundary();
  void setConfig(RoadBoundaryConfig config);
  std::size_t image_size_;

  cv::Point2i convertToIndex(const double x, const double y);
  std::vector<std::optional<cv::Point2i>> findSides(const cv::Point2i current, const cv::Point2i last_point);
  void debugDraw(cv::Point2i point, bool root);
  void drawId(cv::Point2i point, int id);
  void saveImage();
  void drawApproximateLine(std::vector<cv::Point2i> points, int func_id);
  void drawCubicBezier(std::vector<cv::Point2d> controls);
  void fitCurve(std::vector<cv::Point2i> points);
  void writeToOsmFile(const std::vector<std::vector<cv::Point2d>> points);
  std::vector<std::vector<cv::Point2i>> splitLine(std::vector<cv::Point2i> left_points,
                                                  std::vector<cv::Point2i> right_points);
  std::vector<cv::Point2d> convertToGPS(const std::vector<cv::Point2i> points);

private:
  RoadBoundaryConfig config_;
  int func_id_counter_;
  cv::Mat image_;
  cv::Mat debug_image_;
  std::map<cv::Point2i, cv::Point2i, road_boundary::ComparePoints> left_to_right_;
  std::map<cv::Point2i, cv::Point2i, road_boundary::ComparePoints> right_to_left_;
  std::map<cv::Point2i, int, road_boundary::ComparePoints> point_to_func_;
  std::map<int, std::vector<double>> func_id_map_;
  std::map<int, int> lanelet_id_map_;

  cv::Point2d getDirection(const cv::Point2i prev, const cv::Point2i cur);
  std::vector<std::optional<cv::Point2i>> searchPoint(const cv::Point2i current, const cv::Point2d direction);
  std::optional<unsigned char> queryPoint(const cv::Point2i index);
  unsigned char checkPoint(const cv::Point2i index);
  std::vector<std::vector<cv::Point2i>> splitPoints(std::vector<cv::Point2i> points);
};

} // namespace road_boundary

#endif
