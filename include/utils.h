#ifndef _UTILS_H_
#define _UTILS_H_

#include <Eigen/Core>
#include <assert.h>

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

template <typename T>
inline T Square(T value)
{
  return (value * value);
}

template <typename T>
inline T Cube(T value)
{
  return (value * value * value);
}

inline bool DoubleEqual(double a, double b)
{
  double delta = a - b;
  return delta < 0.0 ? delta >= -1e-6 : delta <= 1e-6;
}

template <typename T>
inline bool InRange(const T& value, const T& lower_bound, const T& upper_bound)
{
  return (value >= lower_bound && value <= upper_bound);
}

template <typename T>
inline T Bound(const T& distance, const T& threshold)
{
  if (distance < threshold)
    return threshold - distance;
  else
    return T(0.0);
}

struct Point2D
{
  double x;
  double y;
};

struct Pose2D
{
  double x;
  double y;
  double theta;
};

class MapInfo
{
public:
  inline bool operator==(const MapInfo& map_info) const
  {
    if (width != map_info.width || height != map_info.height || !DoubleEqual(resolution, map_info.resolution) ||
        origin != map_info.origin)
    {
      return false;
    }
    else
      return true;
  }

  inline bool operator!=(const MapInfo& map_info) const
  {
    return !(*this == map_info);
  }

public:
  int width;
  int height;
  double resolution;
  Eigen::Vector2d origin;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // _UTILS_H_