#include "costmap_lite.h"

Costmap2DLite::Costmap2DLite() : binary_map_(NULL), costmap_(NULL), distmap_(NULL)
{
}

Costmap2DLite::Costmap2DLite(double inflation_radius_m)
  : inflation_radius_m_(inflation_radius_m), binary_map_(NULL), costmap_(NULL), distmap_(NULL)
{
  map_info_.width = -1;
  map_info_.height = -1;
  map_info_.resolution = 0.0;
  map_info_.origin = Eigen::Vector2d::Zero();
}

Costmap2DLite::~Costmap2DLite()
{
  freeMemory();
}

void Costmap2DLite::freeMemory()
{
  if (binary_map_)
  {
    delete[] binary_map_;
    binary_map_ = NULL;
  }

  if (costmap_)
  {
    delete[] costmap_;
    costmap_ = NULL;
  }

  if (distmap_)
  {
    delete[] distmap_;
    distmap_ = NULL;
  }
}

void Costmap2DLite::resizeMap(const MapInfo& map_info)
{
  if (map_info_ != map_info)
  {
    freeMemory();

    map_info_ = map_info;

    binary_map_ = new unsigned char[map_info_.width * map_info_.height];
    costmap_ = new double[map_info_.width * map_info_.height];
    distmap_ = new double[map_info_.width * map_info_.height];
  }
}

void Costmap2DLite::updateOccupancyMap(const MapInfo& map_info, const unsigned char* binary_map)
{
  resizeMap(map_info);

  for (int y = 0; y < map_info_.height; ++y)
  {
    for (int x = 0; x < map_info_.width; ++x)
    {
      binary_map_[getIndex(x, y)] = binary_map[getIndex(x, y)];
    }
  }

  computeCostmap();
}

void Costmap2DLite::computeCostmap()
{
  clock_t start_clock = clock();
  cv::Mat gridMapImage(map_info_.height, map_info_.width, CV_8UC1);

  uchar* uchar_ptr = gridMapImage.ptr<uchar>(0);
  for (int i = 0; i < gridMapImage.rows * gridMapImage.cols; ++i)
  {
    if (binary_map_[i] == 1)
    {
      uchar_ptr[i] = 0;
    }
    else if (binary_map_[i] == 0)
    {
      uchar_ptr[i] = 255;
    }
    else
    {
      throw std::runtime_error("The value of occupancy map should be 0 or 1.");
    }
  }

  // calculate the educlidean distance transform via OpenCV distanceTransform function
  cv::Mat distanceFieldImage;
  cv::distanceTransform(gridMapImage, distanceFieldImage, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  float* float_ptr = distanceFieldImage.ptr<float>(0);
  for (int i = 0; i < distanceFieldImage.rows * distanceFieldImage.cols; ++i)
  {
    distmap_[i] = static_cast<double>(float_ptr[i]) * map_info_.resolution;

    costmap_[i] = computeCost(distmap_[i]);
  }
  // printf("time = %f secs\n", double(clock() - start_clock) / CLOCKS_PER_SEC);
}