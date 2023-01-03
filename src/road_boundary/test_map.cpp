#include <iostream>
#include <lanelet2_core/Forward.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_matching/Types.h>
#include <lanelet2_projection/UTM.h>

#include <road_boundary/helper.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

int
main(int argc, char** argv)
{
  const auto yaml = YAML::LoadFile(std::string(argv[1]));
  auto map_origin = readMapOrigin(yaml["map_origin"].as<std::string>());
  lanelet::ErrorMessages errors;
  lanelet::Origin origin({map_origin.center.x, map_origin.center.y});
  auto projector = lanelet::projection::UtmProjector(origin);
  lanelet::io_handlers::OsmParser parser(projector);
  auto map = parser.parse(yaml["map"].as<std::string>(), errors);
  if (errors.empty())
  {
    std::cout << "Loaded successfully" << std::endl;
    std::cout << "Map size " << map->size() << std::endl;
    std::cout << "Lanelet layer size " << map->laneletLayer.size() << std::endl;
  }

  auto resolution = yaml["resolution"].as<double>();

  auto image_size = yaml["image_size"].as<int>();
  auto points = parseBagPath(yaml["test_bag_path"].as<std::string>(),
                             cv::Size(image_size, image_size),
                             resolution,
                             yaml["topic"].as<std::string>());

  bool first_point = false;
  cv::Point2i last_point;
  int count = 0;

  double angle = map_origin.angle;
  double cos = std::cos(angle * M_PI / 180.0);
  double sin = std::sin(angle * M_PI / 180.0);

  for (const auto index : points)
  {
    if (!first_point)
    {
      last_point = index;
      first_point = true;
    }
    if (distance(index, last_point) < yaml["query_distance"].as<double>() / resolution)
    {
      continue;
    }
    // auto gpspoint = projector.reverse(lanelet::BasicPoint3d(
    //     -(index.y * cos + index.x * sin) * resolution, -(index.x * cos - index.y * sin) * resolution, 0));
    // auto normal_point = projector.forward(gpspoint);
    lanelet::matching::Object2d query;
    query.pose.translation() = lanelet::BasicPoint2d {-(index.y * cos + index.x * sin) * resolution,
                                                      -(index.x * cos - index.y * sin) * resolution};
    query.pose.linear() = Eigen::Rotation2D<double>(0.0).matrix();

    auto matching = lanelet::matching::getDeterministicMatches(*map, query, yaml["max_distance"].as<double>());
    if (matching.size() > 0)
    {
      std::cout << "Point " << index.x << " " << index.y << " matches " << matching[0].lanelet.id() << std::endl;
    }
    else
    {
      std::cout << "Point " << index.x << " " << index.y << " has no match\n";
    }
    last_point = index;
    count++;
  }

  std::cout << "Processed " << count << std::endl;
  return 0;
}
