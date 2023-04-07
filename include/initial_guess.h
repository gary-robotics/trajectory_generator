#ifndef TRAJECTORY_GENERATOR_INITIAL_GUESS_H
#define TRAJECTORY_GENERATOR_INITIAL_GUESS_H

#include <vector>
#include "Math.h"

#define t2sTableResolution 0.001
#define t2sTableNum 1001

class InitialGuess
{
public:
  InitialGuess(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4,
               double y4, double x5, double y5);
  ~InitialGuess();

public:
  void getInitialGuess(double& km, double& kf, double& sf) const;

private:
  void t2sTable();
  double tTrans2s(double t) const;
  double sTrans2t(double s) const;

private:
  double x0_, y0_, x1_, y1_, x2_, y2_, x3_, y3_, x4_, y4_, x5_, y5_;
  double sf_;
  double t2s_table_[t2sTableNum];
};

#endif  // TRAJECTORY_GENERATOR_INITIAL_GUESS_H