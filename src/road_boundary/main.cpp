/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#include "road_boundary/node.h"

#include <ros/console.h>
#include <ros/node_handle.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "road_boundary");

  ros::NodeHandle node_handle("~");
  road_boundary::Node road_boundary_node(node_handle);

  ROS_INFO("Initialized");
  ros::spin();
}
