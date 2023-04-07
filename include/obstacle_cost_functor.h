#ifndef _OBSTACLE_COST_FUNCTOR_H_
#define _OBSTACLE_COST_FUNCTOR_H_

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <iostream>
#include "utils.h"
#include "costmap_lite.h"

double bilinearInterpolation(const Eigen::Vector2d& position, const Costmap2DLite* costmap_2d,
                             Eigen::Matrix<double, 1, 2>& M)
{
  const MapInfo map_info = costmap_2d->getMapInfo();
  Eigen::Vector2d point = position - map_info.origin;

  const double x = point.x();
  const double y = point.y();

  int gridX = CONTXY2DISC(x, map_info.resolution);
  int gridY = CONTXY2DISC(y, map_info.resolution);

  double gridCenterX = DISCXY2CONT(gridX, map_info.resolution);
  double gridCenterY = DISCXY2CONT(gridY, map_info.resolution);
  bool inTopLeft = false;
  bool inTopRight = false;
  bool inBottomLeft = false;
  bool inBottomRight = false;

  if (x < gridCenterX && y > gridCenterY)
    inTopLeft = true;
  else if (x >= gridCenterX && y > gridCenterY)
    inTopRight = true;
  else if (x < gridCenterX && y <= gridCenterY)
    inBottomLeft = true;
  else if (x >= gridCenterX && y <= gridCenterY)
    inBottomRight = true;
  else
    throw std::runtime_error("ERROR in bilinearInterpolation: invalid sample");

  int grid_x1, grid_x2, grid_y1, grid_y2;
  if (inTopLeft)
  {
    grid_x1 = gridX - 1;
    grid_x2 = gridX;
    grid_y1 = gridY;
    grid_y2 = gridY + 1;
  }

  if (inTopRight)
  {
    grid_x1 = gridX;
    grid_x2 = gridX + 1;
    grid_y1 = gridY;
    grid_y2 = gridY + 1;
  }

  if (inBottomLeft)
  {
    grid_x1 = gridX - 1;
    grid_x2 = gridX;
    grid_y1 = gridY - 1;
    grid_y2 = gridY;
  }

  if (inBottomRight)
  {
    grid_x1 = gridX;
    grid_x2 = gridX + 1;
    grid_y1 = gridY - 1;
    grid_y2 = gridY;
  }

  double x1 = DISCXY2CONT(grid_x1, map_info.resolution);
  double x2 = DISCXY2CONT(grid_x2, map_info.resolution);
  double y1 = DISCXY2CONT(grid_y1, map_info.resolution);
  double y2 = DISCXY2CONT(grid_y2, map_info.resolution);

  assert(DoubleEqual(x2 - x1, map_info.resolution));
  assert(DoubleEqual(y2 - y1, map_info.resolution));
  assert(InRange(x, x1, x2));
  assert(InRange(y, y1, y2));

  double P11 = costmap_2d->getValue(grid_x1, grid_y1);
  double P21 = costmap_2d->getValue(grid_x2, grid_y1);
  double P12 = costmap_2d->getValue(grid_x1, grid_y2);
  double P22 = costmap_2d->getValue(grid_x2, grid_y2);

  double value = ((y2 - y) * ((x2 - x) * P11 + (x - x1) * P21) + (y - y1) * ((x2 - x) * P12 + (x - x1) * P22)) /
                 (map_info.resolution * map_info.resolution);

  double mx = ((y2 - y) * (P21 - P11) + (y - y1) * (P22 - P12)) / (map_info.resolution * map_info.resolution);
  double my = ((x2 - x) * (P12 - P11) + (x - x1) * (P22 - P21)) / (map_info.resolution * map_info.resolution);
  M << mx, my;

  return value;
}

class ObstacleCostFunction : public ceres::SizedCostFunction<1, 2>
{
public:
  ObstacleCostFunction(const Costmap2DLite* costmap_2d) : costmap_2d_(costmap_2d)
  {
  }

  virtual ~ObstacleCostFunction()
  {
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
  {
    const double x = parameters[0][0];
    const double y = parameters[0][1];

    const Eigen::Vector2d position(x, y);
    Eigen::Matrix<double, 1, 2> M;
    residuals[0] = bilinearInterpolation(position, costmap_2d_, M);

    if (jacobians != NULL && jacobians[0] != NULL)
    {
      jacobians[0][0] = M(0);
      jacobians[0][1] = M(1);
    }

    return true;
  }

private:
  const Costmap2DLite* costmap_2d_;
};

struct ObstacleCostFunctor
{
  ObstacleCostFunctor(const Costmap2DLite* costmap, double inflation_radius_m)
  {
    costmap_2d_ = costmap;
    inflation_radius_m_ = inflation_radius_m;
  }

  template <typename T>
  bool operator()(const T* const x, const T* const y, T* residual) const
  {
    ceres::Grid2D<double, 1, false> grid(costmap_2d_->distmap_, 0, costmap_2d_->map_info_.width, 0,
                                         costmap_2d_->map_info_.height);

    ceres::BiCubicInterpolator<ceres::Grid2D<double, 1, false> > interpolator(grid);

    interpolator.Evaluate(x[0], y[0], &residual[0]);

    residual[0] = Bound(residual[0], T(inflation_radius_m_));

    return true;
  }

  static ceres::CostFunction* create(const Costmap2DLite* costmap, double inflation_radius_m)
  {
    return (new ceres::AutoDiffCostFunction<ObstacleCostFunctor, 1, 1, 1>(
        new ObstacleCostFunctor(costmap, inflation_radius_m)));
  }

  const Costmap2DLite* costmap_2d_;
  double inflation_radius_m_;
};

#endif  // _OBSTACLE_COST_FUNCTOR_H_