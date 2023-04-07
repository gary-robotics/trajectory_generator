#ifndef TRAJECTORY_GENERATOR_MOTION_MODEL_H
#define TRAJECTORY_GENERATOR_MOTION_MODEL_H

#include <cmath>
#include <vector>
#include <Eigen/Eigen>
#include <cstdio>
#include "Math.h"
#include "struct.h"

class MotionModel
{
public:
  MotionModel();
  ~MotionModel();

public:
  void generateTrajectory(double dt, ControlParams& control, TrajectoryProfile& trajectory);
  void generateState(double dt, const LinearVelProfile& linear_vel_profile, double k0, double km, double kf, double sf,
                     double desired_s, Pose& output);

private:
  double estimateDrivingTime(const ControlParams& control) const;
  void makeVelocityProfile(double dt, const LinearVelProfile& linear_vel_profile);
  void calculateSpline(const AngularVelProfile& angular_vel_profile);
  double calculateCubicFunction(double x, const Eigen::Vector4d& coeff) const;
  void update(const State& state, double v, double k, double dt, State& output) const;

private:
  // std::vector<double> v_profile_;
  // std::vector<double> s_profile_;
  double* v_profile_;
  double* s_profile_;
  int size_;

  double ratio_;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > coefficients_;
};

#endif  // TRAJECTORY_GENERATOR_MOTION_MODEL_H