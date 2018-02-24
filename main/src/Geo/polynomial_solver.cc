#include "polynomial_solver.hh"
#include <Eigen/Dense>


namespace Geo {

template<size_t DegT>
std::multiset<double> polygon_roots(const double* _poly)
{
  std::multiset<double> res;
  if constexpr(DegT == 0)
    return res;
  if (_poly[DegT] == 0)
  {
    res = polygon_roots<DegT - 1>(_poly);
    res.insert(0.);
  }
  else
  {
    Eigen::Matrix<double, DegT, DegT> companion;
    companion.setZero();
    companion(0, DegT - 1) = -_poly[0] / _poly[DegT];
    for (int i = 1; i < DegT; ++i)
    {
      companion(i, DegT - 1) = -_poly[i] / _poly[DegT];
      companion(i, i - 1) = 1;
    }
    Eigen::Matrix<std::complex<double>, DegT, 1> roots = companion.eigenvalues();
    for (auto i = roots.rows(); i-- > 0;)
    {
      std::complex<double> val = roots(i, 0);
      if (val.imag() == 0)
        res.insert(val.real());
    }
  }
  return res;
}

template<>
std::multiset<double> polygon_roots<0>(const double*)
{
  return std::multiset<double>();
}

template<size_t DegT>
std::multiset<double> polygon_roots(const std::array<double, DegT + 1>& _poly)
{
  return polygon_roots<DegT>(_poly.data());
}

template std::multiset<double> polygon_roots<2>(const std::array<double, 3>& _poly);
template std::multiset<double> polygon_roots<3>(const std::array<double, 4>& _poly);
//template std::multiset<double> polygon_roots<4>(const std::array<double, 5>& _poly);
//template std::multiset<double> polygon_roots<5>(const std::array<double, 6>& _poly);

} // namespace Geo
