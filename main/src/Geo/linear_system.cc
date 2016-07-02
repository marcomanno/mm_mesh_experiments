#include "linear_system.hh"
#include <cmath>

namespace Geo 
{
// Solution of a 3x3 linear system
bool invert_3x3(const double A[3][3], double iA[3][3])
{
  double det;
  det = A[0][0] * (A[2][2] * A[1][1] - A[2][1] * A[1][2]) - A[1][0] * (A[2][2] * A[0][1] - A[2][1] * A[0][2]) + A[2][0] * (A[1][2] * A[0][1] - A[1][1] * A[0][2]);
  if (std::fabs(det) < 1.e-12)
    return false;

  iA[0][0] = (A[2][2] * A[1][1] - A[2][1] * A[1][2]) / det;
  iA[0][1] = -(A[2][2] * A[0][1] - A[2][1] * A[0][2]) / det;
  iA[0][2] = (A[1][2] * A[0][1] - A[1][1] * A[0][2]) / det;

  iA[1][0] = -(A[2][2] * A[1][0] - A[2][0] * A[1][2]) / det;
  iA[1][1] = (A[2][2] * A[0][0] - A[2][0] * A[0][2]) / det;
  iA[1][2] = -(A[1][2] * A[0][0] - A[1][0] * A[0][2]) / det;

  iA[2][0] = (A[2][1] * A[1][0] - A[2][0] * A[1][1]) / det;
  iA[2][1] = -(A[2][1] * A[0][0] - A[2][0] * A[0][1]) / det;
  iA[2][2] = (A[1][1] * A[0][0] - A[1][0] * A[0][1]) / det;

  return true;
}

bool solve_3x3(const double A[3][3], double x[3], const double b[3])
{
  double iA[3][3];

  if (!invert_3x3(A, iA))
    return false;

  x[0] = iA[0][0] * b[0] + iA[0][1] * b[1] + iA[0][2] * b[2];
  x[1] = iA[1][0] * b[0] + iA[1][1] * b[1] + iA[1][2] * b[2];
  x[2] = iA[2][0] * b[0] + iA[2][1] * b[1] + iA[2][2] * b[2];

  return true;
}

// Solution of a 3x3 linear system
bool invert_2x2(const double A[2][2], double iA[2][2])
{
  double det;
  det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
  if (std::fabs(det) < 1.e-12)
    return false;

  iA[0][0] =  A[1][1] / det;
  iA[0][1] = -A[1][0] / det;
  iA[1][0] = -A[0][1] / det;
  iA[1][1] =  A[0][0] / det;
  return true;
}

bool solve_2x2(const double A[2][2], double x[2], const double b[2])
{
  double iA[2][2];

  if (!invert_2x2(A, iA))
    return false;

  x[0] = iA[0][0] * b[0] + iA[0][1] * b[1];
  x[1] = iA[1][0] * b[0] + iA[1][1] * b[1];

  return true;
}

}// namespace geo