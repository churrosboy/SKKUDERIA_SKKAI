#ifndef GRID_FILTER__GRID_FILTER_HPP_
#define GRID_FILTER__GRID_FILTER_HPP_

#include <opencv2/core.hpp>

#include <filesystem>

namespace grid_filter
{

class GridFilter
{
public:
  bool loadMap(const std::filesystem::path & yaml_path);
  void setErosionKernelSize(int pixels);

  [[nodiscard]] bool isPointInside(double x, double y) const;
  [[nodiscard]] bool isLoaded() const noexcept;

private:
  void updateImage();

  cv::Mat image_;
  cv::Mat eroded_image_;
  double resolution_{0.0};
  cv::Point2d origin_{0.0, 0.0};
  int negate_{0};
  int kernel_size_{1};
};

}  // namespace grid_filter

#endif  // GRID_FILTER__GRID_FILTER_HPP_
