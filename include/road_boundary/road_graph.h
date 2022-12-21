#ifndef ROAD_BOUNDARY_ROAD_GRAPH_H_
#define ROAD_BOUNDARY_ROAD_GRAPH_H_

#include <map>
#include <opencv2/core/types.hpp>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <string>

struct Disc
{
  Disc() {}
  Disc(cv::Point2i c, double r)
    : center {c}
    , radius {r}
  {
  }
  cv::Point2i center;
  double radius;
};

class RouteGraph
{
public:
  RouteGraph(std::string filename);
  bool nearIntersection(cv::Point2i query);

private:
  std::map<int, Disc> centers_;
  std::map<int, std::vector<int>> edges_;
  double max_radius_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_;
};

#endif
