#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <unistd.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

int
main(int argc, char** argv)
{
  const auto yaml = YAML::LoadFile(std::string(argv[1]));
  auto image = cv::imread(yaml["image_path"].as<std::string>(), cv::IMREAD_UNCHANGED);
  cv::Mat eroded_image;
  auto element_size = yaml["erode"]["size"].as<int>();
  auto point = yaml["erode"]["point"].as<int>();
  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(element_size, element_size), cv::Point(point, point));
  cv::erode(image, eroded_image, element);

  auto lane_image = cv::imread(yaml["lane_image_path"].as<std::string>(), cv::IMREAD_UNCHANGED);
  cv::Mat lane_image_resized;
  cv::resize(lane_image, lane_image_resized, eroded_image.size());
  cv::Mat lane_image_masked(
      lane_image_resized.size(), lane_image_resized.type(), cv::Scalar(0, 0, 0, yaml["masked_alpha"].as<int>()));
  lane_image_resized.copyTo(lane_image_masked, eroded_image);

  cv::Mat lane_binary;
  cv::cvtColor(lane_image_masked, lane_binary, cv::COLOR_BGRA2GRAY);
  cv::Mat lane_threshold;
  cv::threshold(lane_binary,
                lane_threshold,
                yaml["threshold"]["value"].as<int>(),
                yaml["threshold"]["max"].as<int>(),
                cv::THRESH_BINARY);
  cv::Mat lane_threshold_type(lane_threshold.size(), CV_8UC1);
  lane_threshold.convertTo(lane_threshold_type, CV_8UC1, 255.0);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(lane_threshold_type, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  std::cout << "Found " << contours.size() << " contours" << std::endl;
  std::vector<std::vector<cv::Point>> chosen;
  for (auto contour : contours)
  {
    auto area = cv::contourArea(contour);
    if (area > yaml["contour"]["min_area"].as<double>())
    {
      chosen.push_back(contour);
    }
  }
  std::cout << "Drawing " << chosen.size() << " contours" << std::endl;
  cv::drawContours(image, chosen, -1, 0, -1);
  cv::Mat eroded_extra;
  cv::erode(image, eroded_extra, element);
  if (!yaml["save_extra_lane"].as<std::string>().empty())
  {
    cv::imwrite(yaml["save_extra_lane"].as<std::string>(), eroded_extra);
  }

  cv::Mat dilated;
  cv::dilate(eroded_image, dilated, element);
  cv::drawContours(dilated, chosen, -1, cv::Scalar(0, 0, 0, yaml["masked_alpha"].as<int>()), -1);

  cv::Mat blurred;
  auto blur_size = yaml["blur"]["kernel"].as<int>();
  cv::GaussianBlur(dilated, blurred, cv::Size(blur_size, blur_size), 0);
  cv::Mat edge;
  cv::Laplacian(blurred,
                edge,
                CV_8U,
                yaml["edge"]["size"].as<int>(),
                yaml["edge"]["scale"].as<int>(),
                yaml["edge"]["delta"].as<int>());
  cv::Mat combined = edge + yaml["lane_alpha"].as<double>() * lane_threshold_type;
  auto saved_image = yaml["save_image"].as<std::string>();
  if (!saved_image.empty())
  {
    cv::imwrite(saved_image, combined);
  }
  return 0;
}
