#ifndef TRAJECTORY_GENERATOR_STRUCT_H
#define TRAJECTORY_GENERATOR_STRUCT_H

#include <Eigen/Eigen>
#include <vector>
#include <cmath>

struct Pose
{
  double x;
  double y;
  double theta;
};

struct State
{
  State(double x_, double y_, double theta_, double v_, double k_)
  {
    pose.x = x_;
    pose.y = y_;
    pose.theta = theta_;
    v = v_;
    k = k_;
  }

  Pose pose;
  double v;
  double k;  // curvature
};

struct LinearVelProfile
{
  double v0;
  double a0;
  double vt;
  double vf;
  double af;
  double time;
};

struct AngularVelProfile
{
  AngularVelProfile()
  {
  }

  AngularVelProfile(double k0_, double km_, double kf_, double sf_)
  {
    k0 = k0_;
    km = km_;
    kf = kf_;
    sf = sf_;
  }

  double k0;
  double km;
  double kf;
  double sf;
};

struct ControlParams
{
  ControlParams()
  {
  }

  ControlParams(const LinearVelProfile& linear, const AngularVelProfile& angular)
  {
    linear_vel_profile = linear;
    angular_vel_profile = angular;
  }

  LinearVelProfile linear_vel_profile;
  AngularVelProfile angular_vel_profile;
};

struct TrajectoryProfile
{
  std::vector<Pose> trajectory;
  std::vector<double> linear;
  std::vector<double> angular;
};

#endif  // TRAJECTORY_GENERATOR_STRUCT_H