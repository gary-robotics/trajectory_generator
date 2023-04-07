#include "trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator(double v0, double k0, double km, double kf, double sf)
  : initialize_(false), model_(NULL)
{
  initialize(v0, k0, km, kf, sf);
}

TrajectoryGenerator::~TrajectoryGenerator()
{
  if (model_)
    delete model_;
}

void TrajectoryGenerator::initialize(double v0, double k0, double km, double kf, double sf)
{
  if (!initialize_)
  {
    dt_ = 0.01;
    tolerance_ = 1e-3;
    max_iteration_ = 100;
    h_ << 0.05, 0.05, 0.1;
    safe_distance_ = 0.3;

    // initialize motion model
    model_ = new MotionModel;

    // initialize control parameters
    control_params_.linear_vel_profile.v0 = v0;
    if (control_params_.linear_vel_profile.v0 < TARGET_VELOCITY)
      control_params_.linear_vel_profile.a0 = MAX_ACCELERATION;
    else
      control_params_.linear_vel_profile.a0 = -MAX_ACCELERATION;

    control_params_.linear_vel_profile.vt = TARGET_VELOCITY;
    control_params_.linear_vel_profile.vf = TARGET_VELOCITY;
    if (control_params_.linear_vel_profile.vt > control_params_.linear_vel_profile.vf)
      control_params_.linear_vel_profile.af = -MAX_ACCELERATION;
    else
      control_params_.linear_vel_profile.af = MAX_ACCELERATION;

    control_params_.linear_vel_profile.time = 0;

    // TODO initialize angular velocity profile
    control_params_.angular_vel_profile.k0 = k0;
    control_params_.angular_vel_profile.km = km;
    control_params_.angular_vel_profile.kf = kf;
    control_params_.angular_vel_profile.sf = sf;

    initialize_ = true;
  }
}

bool TrajectoryGenerator::generateOptimizedTrajectory(
    const Pose& goal, const Eigen::Vector2d& obstacle, TrajectoryProfile& trajectory,
    std::vector<Pose>& samples)
{
  if (!initialize_)
  {
    printf("ERROR: trajectory generator has not been initialized\n");
    return false;
  }

  int iteration = 0;
  // Eigen::Vector3d cost(1e2, 1e2, 1e2);
  // double last_cost = cost.norm();
  // double dist_to_goal = goal.segment(0, 2).norm();
  double last_cost = INFINITECOST;
  bool first_optimization = true;

  while (1)
  {
    if (iteration >= max_iteration_)
    {
      printf("cannot optimize trajectory\n");
      return false;
    }

    // reset
    trajectory.trajectory.clear();
    trajectory.linear.clear();
    trajectory.angular.clear();

    // generate trajectory
    model_->generateTrajectory(dt_, control_params_, trajectory);

    samples.clear();
    for (double ratio = 0.01; ratio <= 0.99; ratio += 0.01)
    {
      Pose sample;
      model_->generateState(dt_, control_params_.linear_vel_profile, control_params_.angular_vel_profile.k0,
                            control_params_.angular_vel_profile.km, control_params_.angular_vel_profile.kf,
                            control_params_.angular_vel_profile.sf, control_params_.angular_vel_profile.sf * ratio,
                            sample);
      samples.push_back(sample);
    }

    // calculate jacobian through central difference linearizations
    Eigen::Matrix3d boundJacobian;
    getBoundJacobian(goal, control_params_, boundJacobian);

    ObsJacobianVector obsJacobians;
    getObstacleJacobians(obstacle, control_params_, obsJacobians);

    Eigen::Vector3d error;
    error(0) = goal.x - trajectory.trajectory.back().x;
    error(1) = goal.y - trajectory.trajectory.back().y;
    error(2) = normalizeAngle(goal.theta - trajectory.trajectory.back().theta);

    Eigen::Matrix3d H = boundJacobian.transpose() * boundJacobian;
    Eigen::Vector3d b = error.transpose() * boundJacobian;
    double cost = error.transpose() * error;

    double max_dist = 0.0;
    int index = -1;
    for (int i = 0; i < (int)samples.size(); i++)
    {
      double dist_cost = obstacleDistanceCost(samples[i], obstacle);
      if (dist_cost > max_dist)
      {
        max_dist = dist_cost;
        index = i;
      }
    }

    H += obsJacobians[index].transpose() * obsJacobians[index];
    b += max_dist * obsJacobians[index];
    cost += max_dist * max_dist;

    Eigen::Vector3d dp = -H.lu().solve(b);

    if (std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) || std::isinf(dp(0)) || std::isinf(dp(1)) ||
        std::isinf(dp(2)) /*|| fabs(dp(2)) > dist_to_goal || cost.norm() > last_cost*/)
    {
      printf("diverge to infinity\n");
      return false;
    }
    // if (!first_optimization)
    // {
    //   if (cost > last_cost)
    //   {
    //     printf("cost > last_cost, diverge to infinity\n");
    //     return false;
    //   }
    // }

    control_params_.angular_vel_profile.km += dp(0);
    control_params_.angular_vel_profile.kf += dp(1);
    control_params_.angular_vel_profile.sf += dp(2);

    if (fabs(cost - last_cost) < tolerance_)
    {
      printf("successfully optimized in %d iteration, cost = %f\n", iteration, cost);
      break;
    }

    first_optimization = false;
    last_cost = cost;
    iteration++;
  }

  return true;
}

void TrajectoryGenerator::getBoundJacobian(const Pose& goal, const ControlParams& control,
                                           Eigen::Matrix3d& jacobian) const
{
  /**
   * h: (dkm, dkf, dsf)
   */

  AngularVelProfile angular_vel = control.angular_vel_profile;
  Pose x0, x1;
  Eigen::Vector3d dx0, dx1;

  model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km - h_(0), angular_vel.kf,
                        angular_vel.sf, angular_vel.sf, x0);
  model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km + h_(0), angular_vel.kf,
                        angular_vel.sf, angular_vel.sf, x1);
  dx0 << goal.x - x0.x, goal.y - x0.y, normalizeAngle
  dx1 = goal - x1;

  Eigen::Vector3d dx_dkm;
  dx_dkm << (dx1 - dx0) / (2.0 * h_(0));

  model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf - h_(1),
                        angular_vel.sf, angular_vel.sf, x0);
  model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf + h_(1),
                        angular_vel.sf, angular_vel.sf, x1);
  dx0 = goal - x0;
  dx1 = goal - x1;

  Eigen::Vector3d dx_dkf;
  dx_dkf << (dx1 - dx0) / (2.0 * h_(1));

  model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf,
                        angular_vel.sf - h_(2), angular_vel.sf - h_(2), x0);
  model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf,
                        angular_vel.sf + h_(2), angular_vel.sf + h_(2), x1);
  dx0 = goal - x0;
  dx1 = goal - x1;

  Eigen::Vector3d dx_dsf;
  dx_dsf << (dx1 - dx0) / (2.0 * h_(2));

  jacobian << dx_dkm(0), dx_dkf(0), dx_dsf(0), dx_dkm(1), dx_dkf(1), dx_dsf(1), dx_dkm(2), dx_dkf(2), dx_dsf(2);
}

void TrajectoryGenerator::getObstacleJacobians(const Eigen::Vector2d& obstacle, const ControlParams& control,
                                               ObsJacobianVector& jacobians) const
{
  AngularVelProfile angular_vel = control.angular_vel_profile;
  Eigen::Vector3d x0, x1;
  double dx0, dx1;

  jacobians.clear();

  for (double ratio = 0.01; ratio <= 0.99; ratio += 0.01)
  {
    model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km - h_(0), angular_vel.kf,
                          angular_vel.sf, angular_vel.sf * ratio, x0);
    model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km + h_(0), angular_vel.kf,
                          angular_vel.sf, angular_vel.sf * ratio, x1);
    dx0 = obstacleDistanceCost(x0, obstacle);
    dx1 = obstacleDistanceCost(x1, obstacle);

    double dx_dkm = (dx1 - dx0) / (2.0 * h_(0));

    model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf - h_(1),
                          angular_vel.sf, angular_vel.sf * ratio, x0);
    model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf + h_(1),
                          angular_vel.sf, angular_vel.sf * ratio, x1);
    dx0 = obstacleDistanceCost(x0, obstacle);
    dx1 = obstacleDistanceCost(x1, obstacle);

    double dx_dkf = (dx1 - dx0) / (2.0 * h_(1));

    model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf,
                          angular_vel.sf - h_(2), (angular_vel.sf - h_(2)) * ratio, x0);
    model_->generateState(dt_, control.linear_vel_profile, angular_vel.k0, angular_vel.km, angular_vel.kf,
                          angular_vel.sf + h_(2), (angular_vel.sf + h_(2)) * ratio, x1);
    dx0 = obstacleDistanceCost(x0, obstacle);
    dx1 = obstacleDistanceCost(x1, obstacle);

    double dx_dsf = (dx1 - dx0) / (2.0 * h_(2));

    Eigen::Matrix<double, 1, 3> jacobian;
    jacobian << dx_dkm, dx_dkf, dx_dsf;
    jacobians.push_back(jacobian);
  }
}

inline double TrajectoryGenerator::obstacleDistanceCost(const Pose& pose,
                                                        const Eigen::Vector2d& obstacle) const
{
  Eigen::Vector2d position;
  position << pose.x, pose.y;
  Eigen::Vector2d dist = position - obstacle;
  if (dist.norm() > safe_distance_)
    return 0.0;
  else
    return (safe_distance_ - dist.norm());
}