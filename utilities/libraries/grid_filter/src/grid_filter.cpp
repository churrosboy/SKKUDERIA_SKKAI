#include <grid_filter/grid_filter.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace grid_filter
{

bool GridFilter::loadMap(const std::filesystem::path & yaml_path)
{
  try {
    const YAML::Node config = YAML::LoadFile(yaml_path.string());
    if (!config["image"] || !config["resolution"] || !config["origin"]) {
      return false;
    }

    const auto origin = config["origin"].as<std::vector<double>>();
    if (origin.size() < 2U) {
      return false;
    }

    resolution_ = config["resolution"].as<double>();
    if (!(resolution_ > 0.0)) {
      return false;
    }
    origin_ = {origin[0], origin[1]};
    negate_ = config["negate"] ? config["negate"].as<int>() : 0;

    std::filesystem::path image_path(config["image"].as<std::string>());
    if (image_path.is_relative()) {
      image_path = yaml_path.parent_path() / image_path;
    }

    image_ = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
    if (image_.empty()) {
      return false;
    }
    cv::flip(image_, image_, 0);
    updateImage();
    return true;
  } catch (const YAML::Exception &) {
    image_.release();
    eroded_image_.release();
    return false;
  }
}

void GridFilter::setErosionKernelSize(int pixels)
{
  if (pixels < 1 || pixels % 2 == 0) {
    throw std::invalid_argument("filter_kernel_size must be a positive odd integer");
  }
  kernel_size_ = pixels;
  updateImage();
}

bool GridFilter::isPointInside(double x, double y) const
{
  if (eroded_image_.empty() || !(resolution_ > 0.0)) {
    return false;
  }

  const int pixel_x = static_cast<int>(std::floor((x - origin_.x) / resolution_));
  const int pixel_y = static_cast<int>(std::floor((y - origin_.y) / resolution_));
  if (pixel_x < 0 || pixel_y < 0 ||
    pixel_x >= eroded_image_.cols || pixel_y >= eroded_image_.rows)
  {
    return false;
  }

  const auto value = eroded_image_.at<unsigned char>(pixel_y, pixel_x);
  return negate_ == 0 ? value >= 250U : value <= 5U;
}

bool GridFilter::isLoaded() const noexcept
{
  return !eroded_image_.empty();
}

void GridFilter::updateImage()
{
  if (image_.empty()) {
    eroded_image_.release();
    return;
  }

  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_));
  if (negate_ == 0) {
    cv::erode(image_, eroded_image_, kernel);
  } else {
    cv::dilate(image_, eroded_image_, kernel);
  }
}

}  // namespace grid_filter
