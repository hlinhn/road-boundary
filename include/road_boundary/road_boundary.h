/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef ROAD_BOUNDARY_ROAD_BOUNDARY_H_
#define ROAD_BOUNDARY_ROAD_BOUNDARY_H_

#include <opencv2/core/mat.hpp>
#include <string>

namespace road_boundary
{
class RoadBoundary
{
public:
  RoadBoundary();

  void setPath(const std::string& path);
  cv::Point2i convertToIndex(const double x, const double y);
  std::pair<std::optional<cv::Point2i>, std::optional<cv::Point2i>> findSides(const cv::Point2i current,
                                                                              const cv::Point2i last_point);
  void debugDraw(cv::Point2i point, bool root);
  void saveImage();
  void drawApproximateLine(std::vector<cv::Point2i> points);

private:
  cv::Mat image_;
  cv::Mat debug_image_;
  std::size_t image_size_;
  double resolution_;
  double max_distance_search_;
  bool read(const std::string& file_path);
  cv::Point2d getDirection(const cv::Point2i prev, const cv::Point2i cur);
  std::optional<cv::Point2i> searchPoint(const cv::Point2i current, const cv::Point2d direction);
};

} // namespace road_boundary

#endif
