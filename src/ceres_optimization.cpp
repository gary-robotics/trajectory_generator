#include <ceres/ceres.h>
#include <cstdio>
#include "motion_model.h"
#include "initial_guess.h"

struct CostFunctor
{
  CostFunctor(const Pose& target, double v0, double k0) : target_(target)
  {
    motion_model_ = new MotionModel;
    control_params_ = new ControlParams;

    // initialize control parameters
    control_params_->linear_vel_profile.v0 = v0;
    if (control_params_->linear_vel_profile.v0 < TARGET_VELOCITY)
      control_params_->linear_vel_profile.a0 = MAX_ACCELERATION;
    else
      control_params_->linear_vel_profile.a0 = -MAX_ACCELERATION;

    control_params_->linear_vel_profile.vt = TARGET_VELOCITY;
    control_params_->linear_vel_profile.vf = TARGET_VELOCITY;
    if (control_params_->linear_vel_profile.vt > control_params_->linear_vel_profile.vf)
      control_params_->linear_vel_profile.af = -MAX_ACCELERATION;
    else
      control_params_->linear_vel_profile.af = MAX_ACCELERATION;

    control_params_->linear_vel_profile.time = 0;

    // TODO initialize angular velocity profile
    control_params_->angular_vel_profile.k0 = k0;

    dt_ = 1e-2;
  }

  ~CostFunctor()
  {
    if (control_params_)
      delete control_params_;
    if (motion_model_)
      delete motion_model_;
  }

  bool operator()(const double* const km, const double* const kf, const double* const sf, double* residual) const
  {
    control_params_->angular_vel_profile.km = km[0];
    control_params_->angular_vel_profile.kf = kf[0];
    control_params_->angular_vel_profile.sf = sf[0];

    Pose last_pose;
    motion_model_->generateState(dt_, control_params_->linear_vel_profile, control_params_->angular_vel_profile.k0,
                                 control_params_->angular_vel_profile.km, control_params_->angular_vel_profile.kf,
                                 control_params_->angular_vel_profile.sf, control_params_->angular_vel_profile.sf,
                                 last_pose);

    residual[0] = target_.x - last_pose.x;
    residual[1] = target_.y - last_pose.y;
    residual[2] = normalizeAngle(target_.theta - last_pose.theta);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Pose& target, double v0, double k0)
  {
    ceres::CostFunction* cost_function =
        new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 3, 1, 1, 1>(new CostFunctor(target, v0, k0));
    return cost_function;
  }

  MotionModel* motion_model_;
  ControlParams* control_params_;
  Pose target_;
  double dt_;
};

int main(int argc, char* argv[])
{
  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.4005;
  double y1 = 0.0;
  double x2 = 0.5671;
  double y2 = -1.2028;
  double x3 = 0.6685;
  double y3 = -1.2231;
  double x4 = 0.9839;
  double y4 = -1.0925;
  double x5 = 1.0607;
  double y5 = -1.0607;

  InitialGuess guess(x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, x5, y5);

  double km, kf, sf;
  guess.getInitialGuess(km, kf, sf);

  printf("km = %f, kf = %f, sf = %f\n", km, kf, sf);

  double v0 = 0;
  double k0 = 0;
  double theta = atan2(y5, x5);
  Pose target;
  target.x = x5;
  target.y = y5;
  target.theta = theta;

  ceres::Problem problem;
  problem.AddResidualBlock(CostFunctor::Create(target, v0, k0), NULL, &km, &kf, &sf);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  return 0;
}