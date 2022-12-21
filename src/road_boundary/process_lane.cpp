#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <unistd.h>

int
main(int argc, char** argv)
{
  auto image = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
  cv::Mat eroded_image;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7), cv::Point(3, 3));
  cv::erode(image, eroded_image, element);
  // cv::imwrite("/home/linh/eroded_image.png", eroded_image);
  auto lane_image = cv::imread(argv[2], cv::IMREAD_UNCHANGED);
  cv::Mat lane_image_resized;
  cv::resize(lane_image, lane_image_resized, eroded_image.size());
  cv::Mat lane_image_masked(lane_image_resized.size(), lane_image_resized.type(), cv::Scalar(0, 0, 0, 255 * 255));
  lane_image_resized.copyTo(lane_image_masked, eroded_image);
  cv::Mat lane_binary;
  cv::cvtColor(lane_image_masked, lane_binary, cv::COLOR_BGRA2GRAY);
  cv::Mat lane_threshold;
  cv::threshold(lane_binary, lane_threshold, 255 * 10.0, 255.0 * 255.0, cv::THRESH_BINARY);
  cv::Mat lane_threshold_type(lane_threshold.size(), CV_8UC1);
  lane_threshold.convertTo(lane_threshold_type, CV_8UC1, 255.0);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(lane_threshold_type, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  std::cout << "Found " << contours.size() << " contours" << std::endl;
  std::vector<std::vector<cv::Point>> chosen;
  std::vector<std::vector<cv::Point>> weaker;
  for (auto contour : contours)
  {
    auto area = cv::contourArea(contour);
    if (area > 60)
    {
      chosen.push_back(contour);
    }
  }
  std::cout << "Drawing " << chosen.size() << " contours" << std::endl;
  std::cout << "Lane lines " << weaker.size() << std::endl;
  cv::drawContours(image, chosen, -1, 0, -1);
  cv::Mat eroded_extra;
  cv::erode(image, eroded_extra, element);
  cv::imwrite("/home/linh/image_with_extra_lanes.png", eroded_extra);

  cv::Mat dilated;
  cv::dilate(eroded_image, dilated, element);

  cv::drawContours(dilated, chosen, -1, cv::Scalar(0, 0, 0, 255 * 255), -1);
  // cv::drawContours(dilated, weaker, -1, cv::Scalar(255 * 0, 255 * 0, 255 * 0, 255 * 255), -1);

  cv::Mat blurred;
  cv::GaussianBlur(dilated, blurred, cv::Size(3, 3), 0);
  cv::Mat edge;
  cv::Laplacian(blurred, edge, CV_8U, 3, 1, 0);
  cv::Mat combined = edge + 0.5 * lane_threshold_type;
  cv::imwrite("/home/linh/inner_region_with_extra.png", combined);

  return 0;
}
