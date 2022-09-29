/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef ROAD_BOUNDARY_ROAD_BOUNDARY_H_
#define ROAD_BOUNDARY_ROAD_BOUNDARY_H_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <string>

namespace road_boundary
{
double pointDistance(cv::Point2i p, cv::Point2i q);
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
  void drawApproximateLine(std::vector<cv::Point2i> points);
  void drawCubicBezier(std::vector<cv::Point2d> controls);
  void fitCurve(std::vector<cv::Point2i> points);

private:
  cv::Mat image_;
  cv::Mat debug_image_;
  std::size_t image_size_;
  bool read(const std::string& file_path);
  cv::Point2d getDirection(const cv::Point2i prev, const cv::Point2i cur);
  std::optional<cv::Point2i> searchPoint(const cv::Point2i current, const cv::Point2d direction);
  std::vector<std::vector<cv::Point2i>> splitPoints(std::vector<cv::Point2i> points);
};

} // namespace road_boundary

#endif
