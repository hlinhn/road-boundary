#include <road_boundary/helper.h>

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

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
  return config;
}

double
distance(cv::Point2i p, cv::Point2i q)
{
  return std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));
};
