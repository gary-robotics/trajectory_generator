#ifndef _OPTIMIZATION_SOLVER_H_
#define _OPTIMIZATION_SOLVER_H_

#include <vector>

#include <ceres/ceres.h>

#include "angle_manifold.h"
#include "costmap_lite.h"
#include "kinematic_cost_function.h"
#include "obstacle_cost_functor.h"
#include "utils.h"

class Solver {
 public:
  Solver() = default;
  virtual ~Solver() = default;

  bool optimize(std::vector<Pose2D>& path, const Costmap2DLite* costmap,
                double inflation_radius_m) {
    int N = (int)path.size();
    if (N < 3) {
      printf(
          "the size of the input is smaller than 3, which cannot be "
          "optimized\n");
      return false;
    }

    double* xs = new double[N];
    double* ys = new double[N];
    double* thetas = new double[N];

    for (int i = 0; i < N; i++) {
      xs[i] = path[i].x / costmap->getMapInfo().resolution;
      ys[i] = path[i].y / costmap->getMapInfo().resolution;
      thetas[i] = path[i].theta;
    }

    // build a problem
    ceres::Problem problem;
    ceres::Manifold* angle_manifold = AngleManifold::Create();

    // add kinematic constraints
    for (int i = 0; i < N - 1; i++) {
      // kinematic constraints
      ceres::CostFunction* kinematic_cost_function =
          KinematicCostFunctor::create();
      problem.AddResidualBlock(kinematic_cost_function, NULL, &xs[i], &ys[i],
                               &thetas[i], &xs[i + 1], &ys[i + 1],
                               &thetas[i + 1]);
      problem.SetManifold(&thetas[i], angle_manifold);
    }
    problem.SetManifold(&thetas[N - 1], angle_manifold);

    // add obstacle constraints
    for (int i = 0; i < N; i++) {
      ceres::CostFunction* obstacle_cost_function =
          ObstacleCostFunctor::create(costmap, inflation_radius_m);
      problem.AddResidualBlock(obstacle_cost_function, NULL, &xs[i], &ys[i]);
    }

    problem.SetParameterBlockConstant(&xs[0]);
    problem.SetParameterBlockConstant(&ys[0]);
    problem.SetParameterBlockConstant(&thetas[0]);
    problem.SetParameterBlockConstant(&xs[N - 1]);
    problem.SetParameterBlockConstant(&ys[N - 1]);
    problem.SetParameterBlockConstant(&thetas[N - 1]);

    // solve the problem
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "Iterations = " << summary.iterations.size() << "\n";

    std::cout << summary.BriefReport() << "\n";

    for (int i = 0; i < N; i++) {
      path[i].x = xs[i] * costmap->getMapInfo().resolution;
      path[i].y = ys[i] * costmap->getMapInfo().resolution;
      path[i].theta = thetas[i];
    }

    delete[] xs;
    delete[] ys;
    delete[] thetas;

    return summary.IsSolutionUsable();
  }
};

#endif  // _OPTIMIZATION_SOLVER_H_