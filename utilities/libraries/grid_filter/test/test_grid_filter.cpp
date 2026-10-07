#include <gtest/gtest.h>

#include <grid_filter/grid_filter.hpp>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <fstream>

class GridFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = std::filesystem::temp_directory_path() / "skk_grid_filter_test";
    std::filesystem::remove_all(root_);
    std::filesystem::create_directories(root_);

    cv::Mat image(7, 7, CV_8UC1, cv::Scalar(255));
    image.at<unsigned char>(3, 3) = 0;
    ASSERT_TRUE(cv::imwrite((root_ / "map.png").string(), image));

    std::ofstream yaml(root_ / "map.yaml");
    yaml << "image: map.png\n"
         << "resolution: 1.0\n"
         << "origin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_);
  }

  std::filesystem::path root_;
};

TEST_F(GridFilterTest, ResolvesRelativeImageAndChecksBounds)
{
  grid_filter::GridFilter filter;
  ASSERT_TRUE(filter.loadMap(root_ / "map.yaml"));
  EXPECT_TRUE(filter.isPointInside(1.0, 1.0));
  EXPECT_FALSE(filter.isPointInside(3.0, 3.0));
  EXPECT_FALSE(filter.isPointInside(-1.0, 1.0));
  EXPECT_FALSE(filter.isPointInside(8.0, 1.0));
}

TEST_F(GridFilterTest, ErosionInflatesOccupiedRegion)
{
  grid_filter::GridFilter filter;
  ASSERT_TRUE(filter.loadMap(root_ / "map.yaml"));
  filter.setErosionKernelSize(3);
  EXPECT_FALSE(filter.isPointInside(2.0, 3.0));
  EXPECT_FALSE(filter.isPointInside(4.0, 3.0));
}

TEST_F(GridFilterTest, RejectsEvenKernel)
{
  grid_filter::GridFilter filter;
  ASSERT_TRUE(filter.loadMap(root_ / "map.yaml"));
  EXPECT_THROW(filter.setErosionKernelSize(2), std::invalid_argument);
}

TEST(GridFilter, MissingMapReturnsFalse)
{
  grid_filter::GridFilter filter;
  EXPECT_FALSE(filter.loadMap("/tmp/skk_grid_filter_missing/map.yaml"));
  EXPECT_FALSE(filter.isLoaded());
}
