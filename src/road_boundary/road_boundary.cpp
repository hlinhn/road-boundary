/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "road_boundary/road_boundary.h"

#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

road_boundary::RoadBoundary::RoadBoundary() {}

void
road_boundary::RoadBoundary::setPath(const std::string& path)
{
  boost::filesystem::path image_path(path);
  image_path = boost::filesystem::canonical(image_path);

  if (!read(image_path.string()))
  {
    throw std::invalid_argument("No such file exists");
  }
}

void
road_boundary::RoadBoundary::saveImage()
{
  cv::imwrite("/home/linh/test_image.png", debug_image_);
}

bool
road_boundary::RoadBoundary::read(const std::string& file_path)
{
  image_ = cv::imread(file_path, cv::IMREAD_UNCHANGED);
  debug_image_ = image_.clone();
  image_size_ = image_.cols;
  resolution_ = 0.1;
  max_distance_search_ = 15;
  return true;
}

cv::Point2i
road_boundary::RoadBoundary::convertToIndex(const double x, const double y)
{
  const auto x_index = (-y + image_size_ * resolution_ / 2.0) / resolution_;
  const auto y_index = (-x + image_size_ * resolution_ / 2.0) / resolution_;
  return cv::Point2i(static_cast<int>(x_index), static_cast<int>(y_index));
}

cv::Point2d
road_boundary::RoadBoundary::getDirection(const cv::Point2i prev, const cv::Point2i cur)
{
  cv::Point2d dir_vec;
  dir_vec.x = cur.x - prev.x;
  dir_vec.y = cur.y - prev.y;
  const auto magnitude = std::sqrt(dir_vec.x * dir_vec.x + dir_vec.y * dir_vec.y);
  dir_vec.x = dir_vec.x / magnitude;
  dir_vec.y = dir_vec.y / magnitude;
  return dir_vec;
}

std::optional<cv::Point2i>
road_boundary::RoadBoundary::searchPoint(const cv::Point2i current, const cv::Point2d direction)
{
  auto max_dx = static_cast<int>(direction.x * max_distance_search_ / resolution_);
  auto max_dy = static_cast<int>(direction.y * max_distance_search_ / resolution_);
  const auto dest_x = current.x + max_dx;
  const auto dest_y = current.y + max_dy;
  const auto sx = max_dx > 0 ? 1 : -1;
  const auto sy = max_dy > 0 ? 1 : -1;
  auto dx = std::abs(max_dx);
  auto dy = -std::abs(max_dy);
  cv::circle(debug_image_, cv::Point2i(dest_x, dest_y), 3, cv::Scalar(0, 255 * 255, 0, 255 * 255), -1);

  cv::Point2i index = current;
  auto error = dx + dy;

  while (true)
  {
    if (index.x >= image_size_ || index.x < 0 || index.y >= image_size_ || index.y < 0)
    {
      break;
    }
    debug_image_.at<cv::Vec<uint16_t, 4>>(index.y, index.x) = cv::Scalar(255 * 255, 255 * 255, 0, 255 * 255);
    const auto value = image_.at<cv::Vec<uint16_t, 4>>(index.y, index.x);
    if (value[0] < static_cast<uint16_t>(62000) || value[1] < 62000 || value[2] < 62000)
    {
      return index;
    }
    if (index.x == dest_x && index.y == dest_y)
    {
      break;
    }
    const auto error_2 = 2 * error;
    if (error_2 >= dy)
    {
      if (index.x == dest_x)
      {
        break;
      }
      error += dy;
      index.x += sx;
    }
    if (error_2 <= dx)
    {
      if (index.y == dest_y)
      {
        break;
      }
      error += dx;
      index.y += sy;
    }
  }
  return std::nullopt;
}

std::pair<std::optional<cv::Point2i>, std::optional<cv::Point2i>>
road_boundary::RoadBoundary::findSides(const cv::Point2i current, const cv::Point2i last_point)
{
  std::optional<cv::Point2i> left_side;
  std::optional<cv::Point2i> right_side;
  auto direction = getDirection(last_point, current);

  cv::Point2d right_vector;
  right_vector.x = -direction.y;
  right_vector.y = direction.x;
  right_side = searchPoint(current, right_vector);

  cv::Point2d left_vector;
  left_vector.x = direction.y;
  left_vector.y = -direction.x;
  left_side = searchPoint(current, left_vector);

  return std::make_pair(left_side, right_side);
}

void
road_boundary::RoadBoundary::debugDraw(cv::Point2i point, bool root)
{
  if (!root)
  {
    cv::circle(debug_image_, point, 3, cv::Scalar(255 * 255, 0, 0, 255 * 255), -1);
  }
  else
  {
    cv::circle(debug_image_, point, 3, cv::Scalar(0, 0, 255 * 255, 255 * 255), -1);
  }
}

void
road_boundary::RoadBoundary::drawApproximateLine(std::vector<cv::Point2i> points)
{
  for (const auto p : points)
  {
    cv::circle(debug_image_, p, 2, cv::Scalar(255 * 255, 0, 255 * 255, 255 * 255));
    // debug_image_.at<cv::Vec<uint16_t, 4>>(p.y, p.x) = cv::Scalar(255 * 255, 0, 255 * 255, 255 * 255);
  }
}
