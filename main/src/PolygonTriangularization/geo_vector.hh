
#pragma once

#include <array>
#include <ostream>

//namespace Geo {

#define VECT_OPERATOR(OP) \
template <typename ValT, size_t N> \
std::array<ValT, N>& operator OP##= (std::array<ValT, N>& _a, const std::array<ValT, N>& _b) \
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

template <typename ValT, size_t N> \
ValT operator*(const std::array<ValT, N>& _a, const std::array<ValT, N>& _b)
{
  ValT dot = 0;
  for (size_t i = N; i-- > 0; dot += _a[i] * _b[i]);
  return dot;
}

template <typename ValT, size_t N> \
ValT length_square(const std::array<ValT, N>& _a)
{
  return _a * _a;
}

template <typename ValT, size_t N> \
ValT length(const std::array<ValT, N>& _a)
{
  return sqrt(length_square(_a));
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
  _os << '[';
  if (_arr.size() > 0)
  {
    for (const auto val : _arr) _os << ' ' << *it;
  }
  _os << ']';
  return _os;
}

namespace Geo
{
typedef std::array<double, 3> Vector3;
}
