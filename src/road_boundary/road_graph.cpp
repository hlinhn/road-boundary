#include <cmath>
#include <iostream>
#include <pcl/point_cloud.h>
#include <road_boundary/road_graph.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

RouteGraph::RouteGraph(std::string filename)
  : max_radius_ {0}
{
  octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128));
  const auto yaml = YAML::LoadFile(filename);
  const auto nodes = yaml["nodes"];
  pcl::PointCloud<pcl::PointXYZ>::Ptr center_clouds(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto data : nodes)
  {
    auto radius = data["radius"].as<double>();
    centers_[data["id"].as<int>()] = Disc(cv::Point2i {data["x"].as<int>(), data["y"].as<int>()}, radius);
    center_clouds->points.emplace_back(data["x"].as<float>(), data["y"].as<float>(), 0);
    if (radius > max_radius_)
    {
      max_radius_ = radius;
    }
  }
  octree_->setInputCloud(center_clouds);
  octree_->addPointsFromInputCloud();

  const auto edges = yaml["edges"];
  for (auto data : edges)
  {
    auto edge_end_points = data["vertices"];
    auto key = edge_end_points[0].as<int>();
    auto end_point = edge_end_points[1].as<int>();
    if (edges_.count(key) == 0)
    {
      std::vector<int> end_points;
      end_points.push_back(end_point);
      edges_[key] = end_points;
    }
    else
    {
      auto end_points = edges_[key];
      end_points.push_back(end_point);
      edges_[key] = end_points;
    }
  }
  std::cout << edges.size() << " " << nodes.size() << std::endl;
}

bool
RouteGraph::nearIntersection(cv::Point2i query)
{
  pcl::PointXYZ search_point(query.x, query.y, 0);
  std::vector<int> point_indices;
  std::vector<float> point_distance;
  if (octree_->radiusSearch(search_point, max_radius_ + 0.1, point_indices, point_distance) > 0)
  {
    for (int i = 0; i < point_indices.size(); i++)
    {
      auto ind = point_indices[i];
      if (std::sqrt(point_distance[i]) <= centers_[ind].radius * 1.42 && edges_.count(ind) > 0
          && edges_[ind].size() > 2)
      {
        return true;
      }
    }
  }
  return false;
}
