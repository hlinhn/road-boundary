#include <iostream>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_matching/Types.h>
#include <lanelet2_projection/UTM.h>

int
main(int argc, char** argv)
{
  lanelet::ErrorMessages errors;
  lanelet::Origin origin({1.3541351067 - 0.0015, 103.695233561 + 0.00491});
  auto projector = lanelet::projection::UtmProjector(origin);
  // lanelet::Origin origin({49.0, 8.4});
  lanelet::io_handlers::OsmParser parser(projector);
  // lanelet::LaneletMapPtr map = lanelet::load("/home/linh/.ros/test_osm_nodes.osm", origin, &errors);
  auto map = parser.parse("/home/linh/.ros/test_osm_nodes.osm", errors);
  if (errors.empty())
  {
    std::cout << "Loaded successfully" << std::endl;
    std::cout << map->size() << std::endl;
    std::cout << map->laneletLayer.size() << std::endl;
  }

  lanelet::matching::Object2d query;
  query.pose.translation() = lanelet::BasicPoint2d {0, 0};
  query.pose.linear() = Eigen::Rotation2D<double>(0.0).matrix();

  auto matching = lanelet::matching::getDeterministicMatches(*map, query, 5000.0);
  std::cout << matching.size() << std::endl;
  for (const auto m : matching)
  {
    std::cout << m.lanelet.id() << " " << m.distance << std::endl;
  }

  auto nearest = map->laneletLayer.nearest(lanelet::BasicPoint2d(0, 0), 1);
  std::cout << nearest[0].id() << std::endl;

  for (const auto data : nearest[0].rightBound2d())
  {
    std::cout << data.x() << " " << data.y() << std::endl;
  }

  return 0;
}
