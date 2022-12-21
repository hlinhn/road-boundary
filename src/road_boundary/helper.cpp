#include <road_boundary/helper.h>

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>

RoadBoundaryConfig
readConfig(std::string filename)
{
  RoadBoundaryConfig config;
  const auto yaml = YAML::LoadFile(filename);
  config.image_path = yaml["image_path"].as<std::string>();
  config.bag_path = yaml["bag_path"].as<std::string>();
  config.cosine_threshold = yaml["cosine_threshold"].as<double>();
  config.distance_threshold = yaml["distance_threshold"].as<double>();
  config.search_threshold = yaml["search_threshold"].as<double>();
  config.split_distance_threshold = yaml["split_distance_threshold"].as<double>();
  config.lane_min_width = yaml["lane_min_width"].as<double>();
  config.min_set_size = yaml["min_set_size"].as<int>();
  config.resolution = yaml["resolution"].as<double>();
  config.route_graph = yaml["route_graph"].as<std::string>();
  config.tolerance = yaml["tolerance"].as<double>();
  config.topic_name = yaml["topic_name"].as<std::string>();
  config.min_time_diff = yaml["min_time_diff"].as<double>();
  config.min_curve_size = yaml["min_curve_size"].as<int>();
  config.min_distance_curve = yaml["min_distance_curve"].as<double>();
  config.save_folder = yaml["save_folder"].as<std::string>();
  config.curb_threshold = yaml["curb_threshold"].as<unsigned int>();
  config.map_origin = yaml["map_origin"].as<std::string>();
  return config;
}

GPSOrigin
readMapOrigin(std::string filename)
{
  GPSOrigin origin;
  const auto yaml = YAML::LoadFile(filename);
  origin.center.x = yaml["center"]["x"].as<double>();
  origin.center.y = yaml["center"]["y"].as<double>();
  origin.angle = yaml["angle"].as<double>();
  return origin;
}

double
distance(cv::Point2i p, cv::Point2i q)
{
  return std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));
};

cv::Point2i
convertToIndex(const double x, const double y, const double resolution, const cv::Size image_size)
{
  const auto x_index = -y / resolution + image_size.height / 2.0;
  const auto y_index = -x / resolution + image_size.width / 2.0;
  return cv::Point2i(static_cast<int>(x_index), static_cast<int>(y_index));
}

std::vector<cv::Point2i>
parseBagPath(std::string bag_path, cv::Size image_size, double resolution, std::string topic)
{
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::BagMode::Read);
  std::vector<std::string> topics;
  topics.push_back(topic);

  std::vector<cv::Point2i> path_points;
  for (rosbag::MessageInstance const message : rosbag::View(bag, rosbag::TopicQuery(topics)))
  {
    const auto position = message.instantiate<nav_msgs::Odometry>();
    const auto index =
        convertToIndex(position->pose.pose.position.x, position->pose.pose.position.y, resolution, image_size);

    path_points.push_back(index);
  }
  return path_points;
}
