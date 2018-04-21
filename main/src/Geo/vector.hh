
#pragma once

#include "tolerance.hh"
#include "pow.hh"
#include "Utils/bindata.hh"

#include <array>

//namespace Geo {

#define VECT_OPERATOR(OP) \
template <typename ValT, size_t N> \
std::array<ValT, N> operator OP##= (std::array<ValT, N>& _a, const std::array<ValT, N>& _b) \
{ \
  for (size_t i = N; i-- > 0; _a[i] OP##= _b[i]); \
  return _a; \
} \
template <typename ValT, size_t N> \
std::array<ValT, N> operator OP (const std::array<ValT, N>& _a, const std::array<ValT, N>& _b) \
{ \
  auto c(_a); \
  return c OP##= _b; \
}

VECT_OPERATOR(+)
VECT_OPERATOR(-)

#define VECT_OPERATOR2(OP) \
template <typename ValT, size_t N> \
std::array<ValT, N>& operator OP##=(std::array<ValT, N>& _a, const ValT& _b) \
{ \
  for (size_t i = N; i-- > 0; _a[i] OP##= _b); \
  return _a; \
} \
template <typename ValT, size_t N> \
std::array<ValT, N> operator OP (const std::array<ValT, N>& _a, const ValT& _b) \
{ \
  auto c(_a); \
  return c OP##= _b; \
} \
template <typename ValT, size_t N> \
std::array<ValT, N> operator OP (const ValT& _a, const std::array<ValT, N>& _b) \
{ \
  auto c(_b); \
  return c OP##= _a; \
}

VECT_OPERATOR2(*)
VECT_OPERATOR2(/)

#undef VECT_OPERATOR
#undef VECT_OPERATOR2

template <typename ValT, size_t N>
std::array<ValT, N> operator-(const std::array<ValT, N>& _a)
{
  auto c(_a);
  for (auto& e : c) e = -e;
  return c;
}

template <typename ValT, size_t N> \
ValT operator*(const std::array<ValT, N>& _a, const std::array<ValT, N>& _b)
{
  ValT dot = 0;
  for (size_t i = N; i-- > 0; dot += _a[i] * _b[i]);
  return dot;
}

template <typename ValT>
std::array<ValT, 3> operator%(const std::array<ValT, 3>& _a, const std::array<ValT, 3>& _b)
{
  std::array<ValT, 3> res;
  res[0] = _a[1] * _b[2] - _a[2] * _b[1];
  res[1] = _a[2] * _b[0] - _a[0] * _b[2];
  res[2] = _a[0] * _b[1] - _a[1] * _b[0];
  return res;
}

template <typename ValT>
ValT operator%(const std::array<ValT, 2>& _a, const std::array<ValT, 2>& _b)
{
  return _a[0] * _b[1] - _a[1] * _b[0];
}

template<typename ValT, size_t N>
std::ostream& operator<<(std::ostream& _os, const std::array<ValT, N>& _arr)
{
  for (const auto& val : _arr)
    _os << ' ' << val; //Utils::BinData<ValT>(val);
  return _os;
}

template<typename ValT, size_t N>
std::istream& operator>>(std::istream& _is, std::array<ValT, N>& _arr)
{
  for (auto& val : _arr)
    _is >> val; //Utils::BinData<ValT>(val);
  return _is;
}

namespace Geo
{
template <typename ValT, size_t N>
ValT length_square(const std::array<ValT, N>& _a)
{
  return _a * _a;
}

template <typename ValT, size_t N>
ValT length(const std::array<ValT, N>& _a)
{
  return sqrt(length_square(_a));
}

template <typename ValT, typename ParametrT>
ValT interpolate(const ValT& _a, const ValT& _b, const ParametrT& _par)
{
  return (1 - _par) * _a + _par * _b;
}

template <typename ValT, size_t N>
bool same(const std::array<ValT, N>& _a, const std::array<ValT, N>& _b, const ValT& _tol)
{
  auto diff_sq = length_square(_a - _b);
  auto tol = std::max(Geo::epsilon_sq(_a), Geo::epsilon_sq(_b));
  if (_tol <= 0)
    return diff_sq <= tol;
  return diff_sq <= std::max(Geo::sq(_tol), tol);
}

template <typename ValT, size_t N>
ValT angle(const std::array<ValT, N>& _a, const std::array<ValT, N>& _b)
{
  return std::atan2(length(_a % _b), _a * _b);
}

template <typename ValT, size_t N>
ValT signed_angle(const std::array<ValT, N>& _a, const std::array<ValT, N>& _b, 
  const std::array<ValT, N>& _norm)
{
  auto cross_vect = _a % _b;
  auto sin_ang = length(cross_vect);
  if (cross_vect * _norm < 0)
    sin_ang = -sin_ang;
  return std::atan2(sin_ang, _a * _b);
}

/*!Finds _u and _v such that ||_w - _u * _a - _v * _b||^2 is minimal.
*/
template<typename ValT, size_t N>
bool decompose(const std::array<ValT, N>& _w,
  const std::array<ValT, N>& _a, const std::array<ValT, N>& _b,
  ValT& _u, ValT& _v);

template <typename ValT, size_t dimT> using Vector = std::array<ValT, dimT>;
template <size_t dimT> using VectorD = Vector<double, dimT>;
typedef VectorD<3> VectorD3;
typedef VectorD<2> VectorD2;

// [a, b, c] is perpendicular to
// [b-c, c-a, a-b]
// [b-c, -a-c, a+b]
// [b+c, c-a, a+b]
// [-b-c, a+c, a-b]
inline void normal_plane_default_directions(
  const Geo::VectorD3& _norm,
  Geo::VectorD3& _du, Geo::VectorD3& _dv)
{
  Geo::VectorD3 nv[] =
  {
    {_norm[1] - _norm[2], _norm[2] - _norm[0], _norm[0] - _norm[1]},
    {_norm[1] - _norm[2], -_norm[2] - _norm[0], _norm[0] + _norm[1]},
    {_norm[1] + _norm[2], _norm[2] - _norm[0], -_norm[0] - _norm[1]},
    {-_norm[1] - _norm[2], _norm[2] + _norm[0], _norm[0] - _norm[1]}
  };
  double val = 0;
  for (auto& du : nv)
  {
    auto val2 = Geo::length_square(du % _norm);
    if (val2 > val)
    {
      val = val2;
      _du = du;
    }
  }
  _dv = _norm % _du;
}

template<size_t DimT>
inline VectorD<DimT> uniform_vector(const double& _v)
{
  VectorD<DimT> vec;
  for (auto& c : vec) c = _v;
  return vec;
}


}//namespace Geo
