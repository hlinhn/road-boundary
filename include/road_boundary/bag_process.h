#ifndef ROAD_BOUNDARY_BAG_PROCESS_H_
#define ROAD_BOUNDARY_BAG_PROCESS_H_

#include "road_boundary/road_boundary.h"
#include <ros/node_handle.h>
namespace road_boundary
{
class BagProcess
{
public:
  BagProcess(ros::NodeHandle& node_handle);

private:
  RoadBoundary road_boundary_;
};
} // namespace road_boundary

#endif
