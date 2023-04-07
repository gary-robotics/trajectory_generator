#include "initial_guess.h"

InitialGuess::InitialGuess(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3,
                           double x4, double y4, double x5, double y5)
{
  x0_ = x0;
  y0_ = y0;
  x1_ = x1;
  y1_ = y1;
  x2_ = x2;
  y2_ = y2;
  x3_ = x3;
  y3_ = y3;
  x4_ = x4;
  y4_ = y4;
  x5_ = x5;
  y5_ = y5;

  t2sTable();
}

InitialGuess::~InitialGuess()
{
}

void InitialGuess::t2sTable()
{
  double s = 0;
  double x_bezier = x0_, y_bezier = y0_;
  int i = 0;
  for (double p = 0; p <= 1 + KT_TOLERANCE; p += t2sTableResolution)
  {
    double x = bezierPoint(x0_, x1_, x2_, x3_, x4_, x5_, p);
    double y = bezierPoint(y0_, y1_, y2_, y3_, y4_, y5_, p);

    s += sqrt(square(x - x_bezier) + square(y - y_bezier));

    t2s_table_[i] = s;

    x_bezier = x;
    y_bezier = y;
    i++;
  }

  sf_ = s;

  assert(i == t2sTableNum);
}

double InitialGuess::tTrans2s(double t) const
{
  int i = int(t * (t2sTableNum - 1));
  assert(i >= 0 && i < t2sTableNum);
  return t2s_table_[i];
}

double InitialGuess::sTrans2t(double s) const
{
  double t1 = 0, t2 = 1;
  double t = 0;
  int i = 0;
  while (1)
  {
    if (i >= 10)
    {
      break;
    }

    t = (t1 + t2) / 2.0;
    double s1 = tTrans2s(t1);
    double s2 = tTrans2s(t2);
    double s_m = tTrans2s(t);
    if (s >= s1 && s <= s_m)
    {
      t2 = t;
    }
    else
    {
      t1 = t;
    }

    i++;
  }

  return t;
}

void InitialGuess::getInitialGuess(double& km, double& kf, double& sf) const
{
  kf = 0;
  sf = sf_;

  double sf_2 = 0.5 * sf_;
  double t = sTrans2t(sf_2);
  km = curvature(x0_, y0_, x1_, y1_, x2_, y2_, x3_, y3_, x4_, y4_, x5_, y5_, t);
}