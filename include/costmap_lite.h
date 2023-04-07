#ifndef COSTMAP_LITE_H
#define COSTMAP_LITE_H

#include <string.h>
#include <stdexcept>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "utils.h"

/**
 * @brief A simplified version of costmap
 */
class Costmap2DLite
{
public:
  Costmap2DLite();
  Costmap2DLite(double inflation_radius_m);
  ~Costmap2DLite();

public:
  inline double getValue(int x, int y) const
  {
    if (!withinMap(x, y))
      std::runtime_error("ERROR: x or y is outside of the map");

    return costmap_[getIndex(x, y)];
  }

  void updateOccupancyMap(const MapInfo& map_info, const unsigned char* binary_map);

  inline const MapInfo& getMapInfo() const
  {
    return map_info_;
  }

private:
  inline int getIndex(int x, int y) const
  {
    return x + y * map_info_.width;
  }

  inline bool withinMap(int x, int y) const
  {
    if (x < 0 || x >= map_info_.width || y < 0 || y >= map_info_.height)
      return false;
    else
      return true;
  }

  inline double computeCost(double distance_m) const
  {
    if (distance_m >= inflation_radius_m_)
      return 0.0;
    else
      return (inflation_radius_m_ - distance_m);
  }

  void computeCostmap();
  void freeMemory();
  void resizeMap(const MapInfo& map_info);

public:
  unsigned char* binary_map_;
  double* costmap_;
  double* distmap_;
  MapInfo map_info_;

  double inflation_radius_m_;
};

#endif  // COSTMAP_LITE_H