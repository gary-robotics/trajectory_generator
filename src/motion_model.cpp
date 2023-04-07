#include "motion_model.h"

MotionModel::MotionModel()
{
  ratio_ = 0.5;
  coefficients_.resize(2);
}

MotionModel::~MotionModel()
{
}

void MotionModel::generateTrajectory(double dt, ControlParams& control, TrajectoryProfile& trajectory)
{
  control.linear_vel_profile.time = estimateDrivingTime(control);

  makeVelocityProfile(dt, control.linear_vel_profile);

  calculateSpline(control.angular_vel_profile);

  double sf_2 = control.angular_vel_profile.sf * ratio_;

  // initial state
  State state(0, 0, 0, control.linear_vel_profile.v0, control.angular_vel_profile.k0);
  State state_(0, 0, 0, control.linear_vel_profile.v0, control.angular_vel_profile.k0);
  Pose pose = state.pose;

  trajectory.trajectory.resize(size_);
  trajectory.linear.resize(size_);
  trajectory.angular.resize(size_);
  trajectory.trajectory[0] = pose;
  trajectory.linear[0] = state.v;
  trajectory.angular[0] = state.v * state.k;

  for (int i = 1; i < size_; i++)
  {
    double s = s_profile_[i];
    double k = 0;
    if (s < sf_2)
    {
      k = calculateCubicFunction(s, coefficients_[0]);
    }
    else
    {
      k = calculateCubicFunction(s, coefficients_[1]);
    }
    update(state, v_profile_[i], k, dt, state_);
    state = state_;
    pose = state.pose;
    trajectory.trajectory[i] = pose;
    trajectory.linear[i] = state.v;
    trajectory.angular[i] = state.v * state.k;
  }
}

void MotionModel::generateState(double dt, const LinearVelProfile& linear_vel_profile, double k0, double km, double kf,
                                double sf, double desired_s, Pose& output)
{
  LinearVelProfile linear_vel = linear_vel_profile;
  AngularVelProfile angular_vel(k0, km, kf, sf);
  ControlParams control(linear_vel, angular_vel);

  linear_vel.time = estimateDrivingTime(control);

  makeVelocityProfile(dt, linear_vel);

  calculateSpline(angular_vel);

  double sf_2 = angular_vel.sf * ratio_;

  State state(0, 0, 0, linear_vel.v0, angular_vel.k0);
  State state_(0, 0, 0, linear_vel.v0, angular_vel.k0);

  for (int i = 1; i < size_; i++)
  {
    double s = s_profile_[i];
    if (s > desired_s)
      break;

    double k = 0;
    if (s < sf_2)
    {
      k = calculateCubicFunction(s, coefficients_[0]);
    }
    else
    {
      k = calculateCubicFunction(s, coefficients_[1]);
    }
    update(state, v_profile_[i], k, dt, state_);
    state = state_;
  }
  output = state.pose;
}

double MotionModel::estimateDrivingTime(const ControlParams& control) const
{
  // acceleration time
  double t0 = (control.linear_vel_profile.vt - control.linear_vel_profile.v0) / control.linear_vel_profile.a0;
  // deceleration time
  double td = (control.linear_vel_profile.vf - control.linear_vel_profile.vt) / control.linear_vel_profile.af;

  double s0 = 0.5 * (control.linear_vel_profile.vt + control.linear_vel_profile.v0) * t0;
  double sd = 0.5 * (control.linear_vel_profile.vt + control.linear_vel_profile.vf) * td;

  double st = control.angular_vel_profile.sf - s0 - sd;
  double tt = st / control.linear_vel_profile.vt;

  return t0 + tt + td;
}

void MotionModel::makeVelocityProfile(double dt, const LinearVelProfile& linear_vel_profile)
{
  size_ = (int)(linear_vel_profile.time / dt);
  if (v_profile_ != NULL)
  {
    delete[] v_profile_;
    v_profile_ = NULL;
  }
  if (s_profile_ != NULL)
  {
    delete[] s_profile_;
    s_profile_ = NULL;
  }

  // v_profile_.resize(size);
  // s_profile_.resize(size);
  v_profile_ = new double[size_];
  s_profile_ = new double[size_];

  printf("size = %d\n", size_);

  double s = 0;
  double t = 0;
  double v_last = 0;
  for (int i = 0; i < size_; ++i)
  {
    // acceleration time
    double ta = (linear_vel_profile.vt - linear_vel_profile.v0) / linear_vel_profile.a0;

    // total time - deceleration time
    double td = linear_vel_profile.time - (linear_vel_profile.vf - linear_vel_profile.vt) / linear_vel_profile.af;

    double v = 0;
    if (t >= 0 && t < ta)
    {
      v = linear_vel_profile.v0 + linear_vel_profile.a0 * t;
    }
    else if (t >= ta && t < td)
    {
      v = linear_vel_profile.vt;
    }
    else if (t >= td && t < linear_vel_profile.time)
    {
      v = linear_vel_profile.vt - linear_vel_profile.af * (t - td);
    }

    v_profile_[i] = v;
    s += v_last * dt;
    v_last = v;
    s_profile_[i] = s;
    t += dt;
  }
}

void MotionModel::calculateSpline(const AngularVelProfile& angular_vel_profile)
{
  // 3d spline interpolation
  Eigen::Vector3d x(0, angular_vel_profile.sf * ratio_, angular_vel_profile.sf);
  Eigen::Vector3d y(angular_vel_profile.k0, angular_vel_profile.km, angular_vel_profile.kf);
  Eigen::Matrix<double, 8, 8> s;
  s << x(0) * x(0) * x(0), x(0) * x(0), x(0), 1, 0, 0, 0, 0, x(1) * x(1) * x(1), x(1) * x(1), x(1), 1, 0, 0, 0, 0, 0, 0,
      0, 0, x(1) * x(1) * x(1), x(1) * x(1), x(1), 1, 0, 0, 0, 0, x(2) * x(2) * x(2), x(2) * x(2), x(2), 1,
      3 * x(1) * x(1), 2 * x(1), 1, 0, -3 * x(1) * x(1), -2 * x(1), -1, 0, 6 * x(1), 2, 0, 0, -6 * x(1), -2, 0, 0,
      6 * x(0), 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6 * x(2), 2, 0, 0;
  Eigen::VectorXd c = Eigen::VectorXd::Zero(8);
  c << y(0), y(1), y(1), y(2), 0, 0, 0, 0;
  Eigen::VectorXd a = s.inverse() * c;
  coefficients_[0] = a.segment(0, 4);
  coefficients_[1] = a.segment(4, 4);
}

double MotionModel::calculateCubicFunction(double x, const Eigen::Vector4d& coeff) const
{
  return coeff(0) * x * x * x + coeff(1) * x * x + coeff(2) * x + coeff(3);
}

void MotionModel::update(const State& state, double v, double k, double dt, State& output) const
{
  output.v = v;
  output.k = k;
  // response_to_control_inputs(s, dt, output_s);

  output.pose.x = state.pose.x + state.v * dt * cos(state.pose.theta);
  output.pose.y = state.pose.y + state.v * dt * sin(state.pose.theta);
  output.pose.theta = normalizeAngle(state.pose.theta + state.v * state.k * dt);
}