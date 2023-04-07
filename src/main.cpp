#include <cstdio>

#include "trajectory_generator.h"
#include "initial_guess.h"

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

  TrajectoryGenerator* generator = new TrajectoryGenerator(v0, k0, km, kf, sf);

  double theta = atan2(y5, x5);
  Pose goal;
  goal.x = x5;
  goal.y = y5;
  goal.theta = theta;
  Eigen::Vector2d obstacle(0.8, -0.2);
  TrajectoryProfile trajectory;
  std::vector<Pose> samples;

  generator->generateOptimizedTrajectory(goal, obstacle, trajectory, samples);

  FILE* file = fopen("../data/trajectory.txt", "w+");
  for (int i = 0; i < (int)trajectory.trajectory.size(); i++)
  {
    fprintf(file, "%f %f %f %f %f\n", trajectory.trajectory[i].x, trajectory.trajectory[i].y,
            trajectory.trajectory[i].theta, trajectory.linear[i], trajectory.angular[i]);
  }
  fclose(file);

  file = fopen("../data/samples.txt", "w+");
  for (int i = 0; i < (int)samples.size(); i++)
  {
    fprintf(file, "%f %f %f\n", samples[i].x, samples[i].y, samples[i].theta);
  }
  fclose(file);

  delete generator;

  return 0;
}