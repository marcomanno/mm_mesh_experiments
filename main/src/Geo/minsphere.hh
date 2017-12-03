#pragma once

#include "Geo/pow.hh"
#include "Geo/linear_system.hh"

namespace Geo {

// Simple sphere class
template <class Point>
struct MinSphereT
{
  MinSphereT() : radius_(-1) {}
  MinSphereT(const Point* _pts, const size_t _pts_num);
  Point centre_;
  double radius_ = -1.;
  bool contains(const Point& _pt) const
  {
    return radius_ >= 0 && length_square(_pt - centre_) <= (1 + Geo::sq(radius_)) * 1e-15;
  }
};

// Constructor from n points
template <class Point>
MinSphereT<Point>::MinSphereT(const Point* _pts, const size_t _pts_num)
{
  radius_ = -1.; // If there is any problem, empty sphere
  if (_pts_num == 0)
    return;

  if (_pts_num == 1)
  {
    radius_ = 0;
    centre_ = _pts[0];
    return;
  }

  if (_pts_num == 2)
  {
    radius_ = std::sqrt(length_square(_pts[0] - _pts[1])) / 2;
    centre_ = (_pts[0] + _pts[1]) * 0.5;
    return;
  }

  if (_pts_num <= 4)
  {
    // Planes intersections. Center will be the intersection
    // of midplanes (p0, plast), (p1, plast), and if 2 != last(p2, plast)
    double A[3][3], B[3];
    size_t points_on_sphere = _pts_num == 3 ? 2 : 3;
    for (size_t i = 0; i < points_on_sphere; ++i)
    {
      B[i] = 0;
      for (int j = 0; j < 3; ++j)
      {
        A[i][j] = _pts[i][j] - _pts[points_on_sphere][j];
        B[i] += (Geo::sq(_pts[i][j]) - Geo::sq(_pts[points_on_sphere][j]));
      }
      B[i] /= 2;
    }
    if (_pts_num == 3)
    {
      // If we have only 3 points the third equation to find
      // the center is the plane for these 3 points.
      auto n = (_pts[0] - _pts[2]) % (_pts[1] - _pts[2]);
      for (int j = 0; j < 3; ++j)
        A[2][j] = n[j];
      B[2] = _pts[2] * n;
    }
    if (solve_3x3(A, centre_.data(), B))
      radius_ = std::sqrt(length_square(_pts[0] - centre_));
    return;
  }
}

template <class Point>
MinSphereT<Point> min_ball(const Point* _pts, const size_t _pts_num)
{
  // Finds the minmum sphere containitn N 3d points.
  Point pts_ball[4];
  size_t pts_ball_num = 0;
  enum class Step {one, two, three};
  std::vector<Step> step_per_depth(_pts_num + 1);
  step_per_depth[_pts_num] = Step::one;
  MinSphereT<Point> sphere;
  for (size_t cur_pts_num = _pts_num; cur_pts_num <= _pts_num; )
  {
    switch(step_per_depth[cur_pts_num])
    {
    case Step::one:
      if (pts_ball_num == 4 || cur_pts_num == 0)
      {
        sphere = MinSphereT<Point>(pts_ball, pts_ball_num);
        // We have the sphere for the first cur_pts_num points.
        // Let's consider one more point.
        cur_pts_num++;
      }
      else
      {
        // Asks for the sphere including the first cur_pts_num - 1
        // points, and after go to step 2.
        step_per_depth[cur_pts_num] = Step::two;
        step_per_depth[--cur_pts_num] = Step::one;
      }
      break;

    case Step::two:
      // Current sphere (*this) is the min ball of cur_pts_num - 1 points.
      // Let's check if it contains also the current point.
      if (sphere.contains(_pts[cur_pts_num - 1]))
        // We have the sphe refor the first cur_pts_num points.
        // Let's consider one more point.
        cur_pts_num++;
      else
      {
        // Current point must be on the bounding sphere.
        // Put it in the list of points where the sphere must lie
        // and compute again the sphere for the first cur_pts_num - 1
        // points.
        pts_ball[pts_ball_num++] = _pts[cur_pts_num - 1];
        step_per_depth[cur_pts_num] = Step::three;
        step_per_depth[--cur_pts_num] = Step::one;
      }
      break;

    case Step::three:
      // Remove the point on sphere added in Step::two
      pts_ball_num--;
      // We have the sphere for the first cur_pts_num points.
      // Let's consider one more point.
      ++cur_pts_num;
      break;
    }
  }
  return sphere;
}

} // namespace Geo
