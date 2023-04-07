#ifndef TRAJECTORY_GENERATOR_MATH_H
#define TRAJECTORY_GENERATOR_MATH_H

#include <cmath>
#include <assert.h>

#define TARGET_VELOCITY 0.8
#define MAX_ACCELERATION 1.0
#define INFINITECOST 1000000000 
const double KT_TOLERANCE = 1e-6;

template <typename T>
inline T square(T value)
{
  return (value * value);
}

inline double normalizeAngle(double angle)
{
  while (angle < -M_PI)
  {
    if (angle < -(2.0 * M_PI))
    {
      angle += (int)(angle / -(2.0 * M_PI)) * (2.0 * M_PI);
    }
    else
    {
      angle += (2.0 * M_PI);
    }
  }

  while (angle > M_PI)
  {
    if (angle > (2.0 * M_PI))
    {
      angle -= (int)(angle / (2.0 * M_PI)) * (2.0 * M_PI);
    }
    else
    {
      angle -= (2.0 * M_PI);
    }
  }

  assert(angle >= -M_PI && angle <= M_PI);

  return angle;
}

// 5-th order bezier curve
inline double bezierPoint(double P0, double P1, double P2, double P3, double P4, double P5, double t)
{
  double value = pow(1 - t, 5) * P0 + 5 * t * pow(1 - t, 4) * P1 + 10 * t * t * pow(1 - t, 3) * P2 +
                 10 * pow(t, 3) * pow(1 - t, 2) * P3 + 5 * pow(t, 4) * (1 - t) * P4 + pow(t, 5) * P5;
  return value;
}
// 5-th order bezier curve first-order derivative
inline double bezierFirstOrder(double P0, double P1, double P2, double P3, double P4, double P5, double t)
{
  double value = 5 * pow(1 - t, 4) * (P1 - P0) + 20 * t * pow(1 - t, 3) * (P2 - P1) +
                 30 * t * t * pow(1 - t, 2) * (P3 - P2) + 20 * pow(t, 3) * (1 - t) * (P4 - P3) +
                 5 * pow(t, 4) * (P5 - P4);
  return value;
}
// 5-th order bezier curve second-order derivative
inline double bezierSecondOrder(double P0, double P1, double P2, double P3, double P4, double P5, double t)
{
  double value = 20 * pow(1 - t, 3) * (P2 - 2 * P1 + P0) + 60 * t * pow(1 - t, 2) * (P3 - 2 * P2 + P1) +
                 60 * t * t * (1 - t) * (P4 - 2 * P3 + P2) + 20 * pow(t, 3) * (P5 - 2 * P4 + P3);
  return value;
}
// 5-th order bezier curve third-order derivative
inline double bezierThirdOrder(double P0, double P1, double P2, double P3, double P4, double P5, double t)
{
  double value = 60 * pow(1 - t, 2) * (P3 - 3 * P2 + 3 * P1 - P0) + 120 * t * (1 - t) * (P4 - 3 * P3 + 3 * P2 - P1) +
                 60 * t * t * (P5 - 3 * P4 + 3 * P3 - P2);
  return value;
}
// curvature
inline double curvature(double x_d, double y_d, double x_dd, double y_dd)
{
  double value = (x_d * y_dd - y_d * x_dd) / pow(x_d * x_d + y_d * y_d, 1.5);
  return value;
}
// curvature first order
inline double curvatureFirstOrder(double x_d, double y_d, double x_dd, double y_dd, double x_ddd, double y_ddd)
{
  double value = ((x_d * y_ddd - y_d * x_ddd) * (x_d * x_d + y_d * y_d) -
                  3 * (x_d * y_dd - y_d * x_dd) * (x_d * x_dd + y_d * y_dd)) /
                 pow(x_d * x_d + y_d * y_d, 2.5);
  return value;
}
// curvature
inline double curvature(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3,
                        double x4, double y4, double x5, double y5, double t)
{
  // first-order derivative
  double x_d = bezierFirstOrder(x0, x1, x2, x3, x4, x5, t);
  double y_d = bezierFirstOrder(y0, y1, y2, y3, y4, y5, t);
  // second-order derivative
  double x_dd = bezierSecondOrder(x0, x1, x2, x3, x4, x5, t);
  double y_dd = bezierSecondOrder(y0, y1, y2, y3, y4, y5, t);
  // curvature
  double k = curvature(x_d, y_d, x_dd, y_dd);
  return k;
}

#endif  // TRAJECTORY_GENERATOR_MATH_H