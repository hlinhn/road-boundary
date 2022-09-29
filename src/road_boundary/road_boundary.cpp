/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "road_boundary/road_boundary.h"
#include "road_boundary/polyfit.hpp"

#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ppl2/ppl.hpp>
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
  params_.resolution = 0.1;
  // max_distance_search_ = 15;
  return true;
}

cv::Point2i
road_boundary::RoadBoundary::convertToIndex(const double x, const double y)
{
  const auto x_index = -y / params_.resolution + image_size_ / 2.0;
  const auto y_index = -x / params_.resolution + image_size_ / 2.0;
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
  auto max_dx = static_cast<int>(direction.x * params_.max_distance_search / params_.resolution);
  auto max_dy = static_cast<int>(direction.y * params_.max_distance_search / params_.resolution);
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
  for (unsigned int i = 0; i < points.size() - 1; i++)
  {
    // auto point = points[i];
    // auto point_next = points[i + 1];
    // if (point.x < 0 || point.y < 0 || point.x > 8000 || point.y > 8000)
    //   continue;
    // if (point_next.x < 0 || point_next.y < 0 || point_next.x > 8000 || point_next.y > 8000)
    //   continue;
    cv::line(debug_image_, points[i], points[i + 1], cv::Scalar(255 * 255, 0, 255 * 255, 255 * 255), 1, cv::LINE_8);
  }
  // for (const auto p : points)
  // {
  //   cv::circle(debug_image_, p, 2, cv::Scalar(255 * 255, 0, 255 * 255, 255 * 255));
  //   // debug_image_.at<cv::Vec<uint16_t, 4>>(p.y, p.x) = cv::Scalar(255 * 255, 0, 255 * 255, 255 * 255);
  // }
}

void
road_boundary::RoadBoundary::drawCubicBezier(std::vector<cv::Point2d> controls)
{
  std::cout << "BEZIER " << std::endl;
  for (const auto p : controls)
  {
    std::cout << p.x << " " << p.y << std::endl;
  }
  std::cout << "-----" << std::endl;
  std::vector<cv::Point2i> points;
  for (int i = 0; i < 21; i++)
  {
    const double t = i * 0.05;
    const double q = 1.0 - t;
    cv::Point2i p;
    p.x = static_cast<int>(q * q * q * controls[0].x + 3 * q * q * t * controls[1].x + 3 * q * t * t * controls[2].x
                           + t * t * t * controls[3].x);
    p.y = static_cast<int>(q * q * q * controls[0].y + 3 * q * q * t * controls[1].y + 3 * q * t * t * controls[2].y
                           + t * t * t * controls[3].y);
    points.push_back(p);
    std::cout << i << " " << p.x << " " << p.y << std::endl;
  }
  drawApproximateLine(points);
}

double
road_boundary::pointDistance(cv::Point2i p, cv::Point2i q)
{
  const auto dx = p.x - q.x;
  const auto dy = p.y - q.y;

  return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::vector<cv::Point2i>>
road_boundary::RoadBoundary::splitPoints(std::vector<cv::Point2i> points)
{
  std::vector<std::vector<cv::Point2i>> point_sets;
  std::vector<cv::Point2i> cur_set;
  cur_set.push_back(points[0]);

  for (int i = 1; i < points.size() - 1; i++)
  {
    auto next = points[i + 1];
    auto prev = points[i - 1];
    auto cur = points[i];

    cv::Point2i pc(cur.x - prev.x, cur.y - prev.y);
    cv::Point2i cn(next.x - cur.x, next.y - cur.y);
    double cosine = (pc.x * cn.x + pc.y * cn.y) / (pointDistance(cur, prev) * pointDistance(next, cur));
    if (cosine < params_.cosine_threshold)
    {
      cur_set.push_back(cur);
      point_sets.push_back(cur_set);
      cur_set.clear();
      continue;
    }
    if (pointDistance(cur, next) > params_.split_distance_threshold)
    {
      cur_set.push_back(cur);
      point_sets.push_back(cur_set);
      cur_set.clear();
      continue;
    }
    if (cur_set.size() == 0 && pointDistance(cur, prev) <= params_.split_distance_threshold)
    {
      cur_set.push_back(prev);
    }
    cur_set.push_back(cur);
  }
  point_sets.push_back(cur_set);
  return point_sets;
}

void
road_boundary::RoadBoundary::fitCurve(std::vector<cv::Point2i> points)
{
  const auto size = points.size();
  if (size < 20)
  {
    return;
  }
  if (pointDistance(points[0], points[size - 1]) < 5.0)
  {
    return;
  }
  auto split_points = splitPoints(points);
  std::vector<std::vector<cv::Point2i>> curves;
  std::cout << "SPLIT INTO " << split_points.size() << std::endl;
  // for (const auto pset : split_points)
  // {
  //   std::cout << "CUR SET " << pset.size() << std::endl;
  //   if (pset.size() < params_.min_set_size)
  //   {
  //     continue;
  //   }
  //   for (const auto p : pset)
  //   {
  //     std::cout << p.x << " " << p.y << std::endl;
  //   }
  //   std::vector<ppl::vertex<double>> data;
  //   for (int i = 0; i < pset.size(); i++)
  //   {
  //     ppl::vertex<double> point2d {pset[i].x * 0.1, pset[i].y * 0.1, 0};
  //     data.push_back(point2d);
  //   }
  //   std::vector<ppl::vertex<double>> control_points;
  //   double tolerance = params_.tolerance;
  //   auto stat = ppl::LERPer::extractB_path(data, control_points, tolerance);

  //   std::cout << "CONTROL POINT " << control_points.size() << std::endl;
  //   const int n = (control_points.size() + 2) / 3 - 1;
  //   for (int i = 0; i < n; i++)
  //   {
  //     std::vector<cv::Point2d> cp;
  //     for (int j = 0; j < 4; j++)
  //     {
  //       cv::Point2d point;
  //       point.x = control_points[i * 3 + j].x * 10.0;
  //       point.y = control_points[i * 3 + j].y * 10.0;
  //       cp.push_back(point);
  //       // debugDraw(point, true);
  //     }
  //     drawCubicBezier(cp);
  //   }
  for (const auto pset : split_points)
  {
    std::cout << "CUR SET " << pset.size() << std::endl;
    if (pset.size() < params_.min_set_size)
    {
      continue;
    }
    for (const auto p : pset)
    {
      std::cout << p.x << " " << p.y << std::endl;
    }
    std::vector<cv::Point2i> output;

    std::vector<double> xs;
    std::vector<double> ys;

    for (int i = 0; i < pset.size(); i++)
    {
      xs.push_back((pset[i].y - 4000) / 4000.0);
      ys.push_back((pset[i].x - 4000) / 4000.0);
    }
    std::vector<double> results = polyfit_Eigen(xs, ys, 7);
    const auto fitted_val = polyval(results, xs);

    for (unsigned int i = 0; i < xs.size(); i++)
    {
      std::cout << xs[i] << " " << ys[i] << " " << fitted_val[i] << std::endl;
      cv::Point2i p;
      p.x = static_cast<int>(fitted_val[i] * 4000.0 + 4000);
      p.y = static_cast<int>(xs[i] * 4000.0 + 4000);
      output.push_back(p);
    }
    drawApproximateLine(output);

    // std::vector<cv::Point2i> output;
    // for (unsigned int i = 0; i < data.size(); i++)
    // {
    //   cv::Point2i p;
    //   p.x = static_cast<int>(data[i].x * 10.0);
    //   p.y = static_cast<int>(data[i].y * 10.0);
    //   output.push_back(p);
    // }

    std::cout << "=============\n";
  }
}
