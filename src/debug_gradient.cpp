#include <cstdio>
#include "optimization_solver.h"

int main(int argc, char* argv[])
{
  FILE* file = fopen("../data/pose.txt", "r");
  double x, y, theta;
  std::vector<Pose2D> path;
  Pose2D pose;

  for (int i = 0; i < 16; i++)
  {
    fscanf(file, "%lf %lf %lf", &x, &y, &theta);
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    path.push_back(pose);
  }
  fclose(file);

  Costmap2DLite* costmap = new Costmap2DLite;
  costmap->map_info_.width = 200;
  costmap->map_info_.height = 200;
  costmap->map_info_.resolution = 0.05;
  costmap->map_info_.origin = Eigen::Vector2d::Zero();
  costmap->distmap_ = new double[costmap->map_info_.width * costmap->map_info_.height];
  file = fopen("../data/maze_dist_map.txt", "r");
  double tmp;
  for (int i = 0; i < costmap->map_info_.width * costmap->map_info_.height; i++)
  {
    fscanf(file, "%lf", &tmp);
    costmap->distmap_[i] = tmp;
  }

  double inflation_radius_m = 0.5;
  Solver* solver = new Solver();
  solver->optimize(path, costmap, inflation_radius_m);

  file = fopen("../data/optimized_path.txt", "w+");
  for (std::vector<Pose2D>::const_iterator iter = path.begin(); iter != path.end(); ++iter)
  {
    fprintf(file, "%lf %lf %lf\n", iter->x, iter->y, iter->theta);
  }
  fclose(file);

  delete solver;
  delete costmap;

  return 0;
}