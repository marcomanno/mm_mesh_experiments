
#include "vector.hh"
#include "linear_system.hh"
#include <Eigen/Dense>


namespace Geo {

/*!Finds _u and _v such that ||_w - _u * _a - _v * _b||^2 is minimal.
*/
template<typename ValT, size_t N>
bool decompose(const std::array<ValT, N>& _w,
  const std::array<ValT, N>& _a, const std::array<ValT, N>& _b,
  ValT& _u, ValT& _v)
{
  Eigen::MatrixXd A(N, 2);
  Eigen::VectorXd B(N);
  for (auto i = 0; i < N; ++i)
  {
    A(i, 0) = _a[i];
    A(i, 1) = _b[i];
    B[i] = _w[i];
  }
  Eigen::VectorXd res = A.colPivHouseholderQr().solve(B);
  _u = res[0];
  _v = res[1];
  return true;
}

template bool decompose(const std::array<double, 3>& _w,
  const std::array<double, 3>& _a, const std::array<double, 3>& _b,
  double& _u, double& _v);

template bool decompose(const std::array<double, 2>& _w,
  const std::array<double, 2>& _a, const std::array<double, 2>& _b,
  double& _u, double& _v);

}//namespace Geo
