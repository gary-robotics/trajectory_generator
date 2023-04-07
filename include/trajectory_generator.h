#ifndef TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H

#include <cstdio>
#include <Eigen/Dense>
#include "Math.h"
#include "struct.h"
#include "motion_model.h"

typedef std::vector<Eigen::Matrix<double, 1, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 3> > >
    ObsJacobianVector;

class TrajectoryGenerator
{
public:
  TrajectoryGenerator(double v0, double k0, double km, double kf, double sf);
  ~TrajectoryGenerator();

public:
  bool generateOptimizedTrajectory(const Pose& goal, const Eigen::Vector2d& obstacle,
                                   TrajectoryProfile& trajectory,
                                   std::vector<Pose>& samples);

private:
  void initialize(double v0, double k0, double km, double kf, double sf);
  void getBoundJacobian(const Pose& goal, const ControlParams& control, Eigen::Matrix3d& jacobian) const;
  void getObstacleJacobians(const Eigen::Vector2d& obstacle, const ControlParams& control,
                            ObsJacobianVector& jacobians) const;
  inline double obstacleDistanceCost(const Pose& pose, const Eigen::Vector2d& obstacle) const;

private:
  MotionModel* model_;
  ControlParams control_params_;

  double dt_;
  double tolerance_;
  int max_iteration_;

  Eigen::Vector3d h_;
  double safe_distance_;
  bool initialize_;
};

#endif  // TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H