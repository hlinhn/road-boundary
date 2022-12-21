/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "road_boundary/helper.h"
#include "road_boundary/polyfit.hpp"
#include <road_boundary/road_boundary.h>

#include <algorithm>
#include <bits/stdint-uintn.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>
#include <math.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <osmium/builder/attr.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/osm/item_type.hpp>
#include <osmium/osm/location.hpp>
#include <ppl2/ppl.hpp>
#include <stdexcept>
#include <string>

road_boundary::RoadBoundary::RoadBoundary()
  : func_id_counter_ {0}
{
}

void
road_boundary::RoadBoundary::setConfig(RoadBoundaryConfig config)
{
}

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
  params_.resolution = 0.25;
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

std::optional<unsigned char>
road_boundary::RoadBoundary::queryPoint(const cv::Point2i index)
{
  if (index.x >= image_size_ || index.x < 0 || index.y >= image_size_ || index.y < 0)
  {
    return std::nullopt;
  }
  return image_.at<unsigned char>(index.y, index.x);
}

unsigned char
road_boundary::RoadBoundary::checkPoint(const cv::Point2i index)
{
  std::vector<cv::Point2i> neighbors;
  for (int i = -1; i < 2; i++)
  {
    neighbors.emplace_back(index.x + i, index.y + i);
    if (i != 0)
    {
      neighbors.emplace_back(index.x + i, index.y - i);
      neighbors.emplace_back(index.x, index.y + i);
      neighbors.emplace_back(index.x + i, index.y);
    }
  }
  for (auto point : neighbors)
  {
    auto value_optional = queryPoint(point);
    if (!value_optional)
    {
      continue;
    }
    auto value = value_optional.value();
    if (value > 0)
    {
      return value;
    }
  }
  return 0;
}

std::vector<std::optional<cv::Point2i>>
road_boundary::RoadBoundary::searchPoint(const cv::Point2i current, const cv::Point2d direction)
{
  std::vector<std::optional<cv::Point2i>> points;
  auto max_dx = static_cast<int>(direction.x * params_.max_distance_search / params_.resolution);
  auto max_dy = static_cast<int>(direction.y * params_.max_distance_search / params_.resolution);
  const auto dest_x = current.x + max_dx;
  const auto dest_y = current.y + max_dy;
  const auto sx = max_dx > 0 ? 1 : -1;
  const auto sy = max_dy > 0 ? 1 : -1;
  auto dx = std::abs(max_dx);
  auto dy = -std::abs(max_dy);
  // cv::circle(debug_image_, cv::Point2i(dest_x, dest_y), 3, cv::Scalar(255), -1);

  cv::Point2i index = current;
  auto error = dx + dy;

  while (true)
  {
    if (index.x >= image_size_ || index.x < 0 || index.y >= image_size_ || index.y < 0)
    {
      break;
    }
    debug_image_.at<unsigned char>(index.y, index.x) = 255;
    auto value = checkPoint(index);
    if (value >= 200)
    {
      points.push_back(index);
      return points;
    }
    else if (value > 0)
    {
      points.push_back(index);
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
  points.push_back(std::nullopt);
  return points;
}

std::vector<std::optional<cv::Point2i>>
road_boundary::RoadBoundary::findSides(const cv::Point2i current, const cv::Point2i last_point)
{
  auto direction = getDirection(last_point, current);

  cv::Point2d right_vector;
  right_vector.x = -direction.y;
  right_vector.y = direction.x;
  auto right_side = searchPoint(current, right_vector);

  cv::Point2d left_vector;
  left_vector.x = direction.y;
  left_vector.y = -direction.x;
  auto left_side = searchPoint(current, left_vector);

  // if (right_side.has_value() && left_side.has_value())
  // {
  //   left_to_right_[left_side.value()] = right_side.value();
  //   right_to_left_[right_side.value()] = left_side.value();
  // }

  std::reverse(left_side.begin(), left_side.end());
  left_side.insert(left_side.end(), right_side.begin(), right_side.end());
  return left_side;
}

void
road_boundary::RoadBoundary::debugDraw(cv::Point2i point, bool root)
{
  if (!root)
  {
    cv::circle(debug_image_, point, 3, cv::Scalar(255), -1);
  }
  else
  {
    cv::circle(debug_image_, point, 3, cv::Scalar(150), -1);
  }
}

void
road_boundary::RoadBoundary::drawId(cv::Point2i point, int id)
{
  // cv::circle(debug_image_, point, 3, cv::Scalar(255), -1);
  // cv::putText(debug_image_, std::to_string(id), point, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(150), 1, cv::LINE_8);
}

void
road_boundary::RoadBoundary::drawApproximateLine(std::vector<cv::Point2i> points, int func_id)
{
  for (unsigned int i = 0; i < points.size() - 1; i++)
  {
    cv::line(debug_image_, points[i], points[i + 1], cv::Scalar(255), 1, cv::LINE_8);
  }
  // cv::putText(
  //     debug_image_, std::to_string(func_id), points[0], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(150), 1, cv::LINE_8);
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
  drawApproximateLine(points, 0);
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
    double cosine = (pc.x * cn.x + pc.y * cn.y) / (distance(cur, prev) * distance(next, cur));
    if (cosine < params_.cosine_threshold)
    {
      cur_set.push_back(cur);
      point_sets.push_back(cur_set);
      cur_set.clear();
      continue;
    }
    if (distance(cur, next) > params_.split_distance_threshold)
    {
      cur_set.push_back(cur);
      point_sets.push_back(cur_set);
      cur_set.clear();
      continue;
    }
    if (cur_set.size() == 0 && distance(cur, prev) <= params_.split_distance_threshold)
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
  if (size < 10)
  {
    return;
  }
  if (distance(points[0], points[size - 1]) < 5.0)
  {
    return;
  }
  auto split_points = splitPoints(points);
  std::vector<std::vector<cv::Point2i>> curves;
  std::cout << "SPLIT INTO " << split_points.size() << std::endl;
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
      xs.push_back((pset[i].y - 1600) / 1600.0);
      ys.push_back((pset[i].x - 1600) / 1600.0);
    }
    std::vector<double> results = polyfit_Eigen(xs, ys, 7);
    const auto fitted_val = polyval(results, xs);
    func_id_counter_++;
    func_id_map_[func_id_counter_] = results;

    for (unsigned int i = 0; i < xs.size(); i++)
    {
      point_to_func_[pset[i]] = func_id_counter_;
      std::cout << xs[i] << " " << ys[i] << " " << fitted_val[i] << std::endl;
      cv::Point2i p;
      p.x = static_cast<int>(fitted_val[i] * 1600.0 + 1600);
      p.y = static_cast<int>(xs[i] * 1600.0 + 1600);
      output.push_back(p);
    }
    drawApproximateLine(output, func_id_counter_);
    std::cout << "=============\n";
  }
}

std::vector<cv::Point2d>
road_boundary::RoadBoundary::convertToGPS(const std::vector<cv::Point2i> points)
{
  std::vector<cv::Point2d> gps_points;
  cv::Point2d center;
  center.x = 1.3541351067 - 0.0015;
  center.y = 103.695233561 + 0.00491;

  double angle = 62;
  double cos = std::cos(angle * M_PI / 180.0);
  double sin = std::sin(angle * M_PI / 180.0);

  auto projector = lanelet::projection::UtmProjector(lanelet::Origin({center.x, center.y}));
  for (const auto point : points)
  {
    cv::Point2d rotated;
    rotated.x = (point.x * cos - point.y * sin) * params_.resolution;
    rotated.y = (point.y * cos + point.x * sin) * params_.resolution;
    cv::Point2d gps;
    auto gpspoint = projector.reverse(lanelet::BasicPoint3d(-rotated.y, -rotated.x, 0));
    gps.x = gpspoint.lat;
    gps.y = gpspoint.lon;
    // gps.x = center.x - 0.1 * static_cast<double>(rotated.x) / 111320.0;
    // gps.y = center.y - 0.1 * static_cast<double>(rotated.y) / (111320.0 * std::cos(M_PI * gps.x / 180.0));
    gps_points.push_back(gps);
  }
  return gps_points;
}

std::vector<std::vector<cv::Point2i>>
road_boundary::RoadBoundary::splitLine(std::vector<cv::Point2i> left_points, std::vector<cv::Point2i> right_points)
{
  std::vector<std::vector<cv::Point2i>> split;
  std::vector<cv::Point2i> cur_left;
  std::vector<cv::Point2i> cur_right;
  int cur_left_func = -1;
  int cur_right_func = -1;
  int last_left_id = -1;
  std::map<cv::Point2i, int, road_boundary::ComparePoints> point_to_index;

  for (const auto p : left_points)
  {
    if (point_to_func_.count(p) == 0)
    {
      continue;
    }
    int id = point_to_func_[p];
    if (cur_left_func == -1)
    {
      cur_left_func = id;
    }
    if (id != cur_left_func)
    {
      if (cur_left.size() > 1)
      {
        for (const auto pl : cur_left)
        {
          point_to_index[pl] = split.size();
        }
        split.push_back(cur_left);
      }
      cur_left.clear();
      cur_left_func = id;
      continue;
    }

    if (left_to_right_.count(p) > 0)
    {
      const auto corres = left_to_right_[p];
      if (point_to_func_.count(corres) > 0)
      {
        int corres_id = point_to_func_[corres];
        if (cur_right_func == -1)
        {
          cur_right_func = corres_id;
        }
        if (corres_id != cur_right_func)
        {
          if (cur_left.size() > 1)
          {
            for (const auto pl : cur_left)
            {
              point_to_index[pl] = split.size();
            }
            split.push_back(cur_left);
          }
          cur_left.clear();
          cur_right_func = corres_id;
        }
      }
    }
    cur_left.push_back(p);
  }
  if (cur_left.size() > 1)
  {
    for (const auto pl : cur_left)
    {
      point_to_index[pl] = split.size();
    }
    split.push_back(cur_left);
  }

  last_left_id = split.size() - 1;
  cur_left_func = -1;
  cur_right_func = -1;
  for (const auto p : right_points)
  {
    if (point_to_func_.count(p) == 0)
    {
      continue;
    }
    int id = point_to_func_[p];
    if (cur_right_func == -1)
    {
      cur_right_func = id;
    }
    if (id != cur_right_func)
    {
      if (cur_right.size() > 1)
      {
        for (const auto pr : cur_right)
        {
          point_to_index[pr] = split.size();
        }
        split.push_back(cur_right);
      }
      cur_right.clear();
      cur_right_func = id;
      continue;
    }

    if (right_to_left_.count(p) > 0)
    {
      const auto corres = right_to_left_[p];
      if (point_to_func_.count(corres) > 0)
      {
        int corres_id = point_to_func_[corres];
        if (cur_left_func == -1)
        {
          cur_left_func = corres_id;
        }
        if (corres_id != cur_left_func)
        {
          if (cur_right.size() > 1)
          {
            for (const auto pr : cur_right)
            {
              point_to_index[pr] = split.size();
            }
            split.push_back(cur_right);
          }
          cur_right.clear();
          cur_left_func = corres_id;
        }
      }
    }
    cur_right.push_back(p);
  }
  if (cur_right.size() > 1)
  {
    for (const auto pr : cur_right)
    {
      point_to_index[pr] = split.size();
    }
    split.push_back(cur_right);
  }

  for (int i = 0; i <= last_left_id; i++)
  {
    const auto pl = split[i];
    for (const auto p : pl)
    {
      if (left_to_right_.count(p) > 0 && point_to_index.count(left_to_right_[p]) > 0)
      {
        std::cout << left_to_right_[p].x << " " << left_to_right_[p].y << std::endl;
        std::cout << point_to_index[p] << " " << point_to_index[left_to_right_[p]] << std::endl;
        lanelet_id_map_[i] = point_to_index[left_to_right_[p]];
        break;
      }
    }
  }
  return split;
}

std::optional<cv::Point2i>
road_boundary::RoadBoundary::projectPoint(cv::Point2i point)
{
  if (point_to_func_.count(point) == 0)
  {
    std::cout << "This point does not belong to one of the original function" << std::endl;
    return std::nullopt;
  }
  auto func = func_id_map_[point_to_func_[point]];
  double slope = func[1];
  double orig_x_var = (point.y - 4000) / 4000.0;
  double cur_x_var = orig_x_var;
  for (size_t i = 2; i < func.size(); i++)
  {
    slope += i * func[i] * cur_x_var;
    cur_x_var *= orig_x_var;
  }
  if (std::abs(slope) < 1e-5)
  {
    std::cout << "Slope too close to 0" << std::endl;
    return std::nullopt;
  }
  auto new_slope = -1 / slope;
  // solve 7th order polynomial?
  return std::nullopt;
}

void
road_boundary::RoadBoundary::testROI(std::vector<std::vector<cv::Point2i>> lines)
{
  auto lane_image = cv::imread("/home/linh/Downloads/bags/cleantech/lane_grid_map_drivable.png", cv::IMREAD_UNCHANGED);
  cv::Mat new_lane(lane_image.rows, lane_image.cols, lane_image.type());
  std::vector<std::vector<cv::Point2i>> contours;
  cv::Mat mask = cv::Mat::zeros(lane_image.rows, lane_image.cols, CV_8U);
  for (const auto idp : lanelet_id_map_)
  {
    auto left = lines[idp.first];
    auto right = lines[idp.second];
    std::reverse(right.begin(), right.end());
    left.insert(left.end(), right.begin(), right.end());
    contours.push_back(left);
    // break;
  }
  cv::fillPoly(mask, contours, cv::Scalar(1));
  lane_image.copyTo(new_lane, mask);
  cv::imwrite("/home/linh/test_lane_image.png", new_lane);

  cv::Mat blur_im;
  cv::Mat canny;
  cv::Mat output;
  new_lane.convertTo(blur_im, CV_8U);
  cv::blur(blur_im, canny, cv::Size(5, 5));
  cv::Canny(canny, output, 0, 1);
  std::vector<cv::Vec3f> detected;

  cv::HoughLines(output, detected, 1, CV_PI / 180, 70, 0, 0);
  std::cout << "DETECTED " << detected.size() << std::endl;

  for (auto l : detected)
  {
    auto theta = l[1];
    auto rho = l[0];
    auto vote = l[2];
    auto a = cos(theta);
    auto b = sin(theta);
    auto x = a * rho;
    auto y = b * rho;
    auto p1n = cv::Point(static_cast<int>(x + 4000 * -b), static_cast<int>(y + 4000 * a));
    auto p2n = cv::Point(static_cast<int>(x + 4000 * b), static_cast<int>(y - 4000 * a));
    cv::line(output, p1n, p2n, cv::Scalar(255), 1, cv::LINE_AA);
  }
  cv::imwrite("/home/linh/test_lane_image_imm.png", output);
}

void
road_boundary::RoadBoundary::writeToOsmFile(const std::vector<std::vector<cv::Point2d>> points)
{
  const osmium::io::File output_file {"test_osm_nodes.osm", "xml"};
  osmium::io::Header header;
  header.set("generator", "test_nodes");

  const size_t initial_buffer_size = 10000;
  osmium::memory::Buffer buffer {initial_buffer_size, osmium::memory::Buffer::auto_grow::yes};
  using namespace osmium::builder::attr;
  long node_id = 0;
  osmium::io::Writer writer {output_file, header, osmium::io::overwrite::allow};
  int way_id = 1;
  for (const auto nodes : points)
  {
    std::vector<long> id_in_line;
    for (size_t i = 0; i < nodes.size(); i++)
    {
      osmium::builder::add_node(buffer,
                                _id(node_id + 1),
                                _version(1),
                                _timestamp(std::time(nullptr)),
                                _location(osmium::Location {nodes[i].y, nodes[i].x}));
      id_in_line.push_back(node_id + 1);
      node_id++;
    }
    osmium::builder::add_way(buffer,
                             _id(way_id),
                             _version(1),
                             _timestamp(std::time(nullptr)),
                             _tags({{"type", "curbstone"}}),
                             _nodes(id_in_line));
    way_id++;
  }
  int lanelet_id = 1;
  for (const auto idp : lanelet_id_map_)
  {
    std::cout << idp.first << " " << idp.second << std::endl;
    osmium::builder::add_relation(buffer,
                                  _id(lanelet_id),
                                  _version(1),
                                  _timestamp(std::time(nullptr)),
                                  _tags({{"type", "lanelet"}}),
                                  _member(osmium::item_type::way, idp.first + 1, "left"),
                                  _member(osmium::item_type::way, idp.second + 1, "right"));
    lanelet_id++;
  }
  writer(std::move(buffer));
  writer.close();
}
