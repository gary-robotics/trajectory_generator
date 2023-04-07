#ifndef _KINEMATIC_COST_FUNCTION_H_
#define _KINEMATIC_COST_FUNCTION_H_

#include <ceres/ceres.h>

struct KinematicCostFunctor
{
  template <typename T>
  bool operator()(const T* const x0, const T* const y0, const T* const theta0, const T* const x1, const T* const y1,
                  const T* const theta1, T* residual) const
  {
    const T d0 = x1[0] - x0[0];
    const T d1 = y1[0] - y0[0];

    residual[0] =
        (ceres::cos(theta0[0]) + ceres::cos(theta1[0])) * d1 - (ceres::sin(theta0[0]) + ceres::sin(theta1[0])) * d0;

    return true;
  }

  static ceres::CostFunction* create()
  {
    return (new ceres::AutoDiffCostFunction<KinematicCostFunctor, 1, 1, 1, 1, 1, 1, 1>(new KinematicCostFunctor));
  }
};

#endif  // _KINEMATIC_COST_FUNCTION_H_