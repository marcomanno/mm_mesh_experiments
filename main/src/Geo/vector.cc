
#include "vector.hh"
#include "linear_system.hh"

namespace Geo {

/*!Finds _u and _v such that ||_w - _u * _a - _v * _b||^2 is minimal.
*/
template<typename ValT, size_t N>
bool decompose(const std::array<ValT, N>& _w,
  const std::array<ValT, N>& _a, const std::array<ValT, N>& _b,
  ValT& _u, ValT& _v)
{
  ValT A[2][2], B[2], X[2];
  A[0][0] = _a * _a;
  A[1][0] = A[0][1] = _a * _b;
  A[1][1] = _b * _b;
  B[0] = _w * _a;
  B[1] = _w * _b;

  if (!solve_2x2(A, X, B))
    return false;
  _u = X[0];
  _v = X[1];
  return true;
}

template bool decompose(const std::array<double, 3>& _w,
  const std::array<double, 3>& _a, const std::array<double, 3>& _b,
  double& _u, double& _v);

template bool decompose(const std::array<double, 2>& _w,
  const std::array<double, 2>& _a, const std::array<double, 2>& _b,
  double& _u, double& _v);

}//namespace Geo
