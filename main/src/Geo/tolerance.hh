#pragma once

#include "vector.hh"

#include <array>
#include <limits>

namespace Geo {

constexpr double  reference_unit()
{
  return 0.0001;
}

template <typename ValueT>
constexpr double  precision_sq()
{
  return std::numeric_limits<ValueT>::epsilon() * 10;
}

template <typename ValueT>
constexpr double  precision()
{
  return sqrt(precision_sq<ValueT>());
}

template <typename ValueT>
ValueT epsilon(const ValueT& _ref)
{
  return (std::abs(_ref) + reference_unit()) * precision<ValueT>();
}

template <typename ValueT>
ValueT epsilon_sq(const ValueT& _ref)
{
  return Geo::sq(epsilon(_ref));
}

template <typename ValueT, size_t N>
ValueT epsilon_sq(const std::array<ValueT, N>& _ref)
{
  return epsilon_sq(length(_ref));
}

template <typename ValueT, size_t N>
ValueT epsilon(const std::array<ValueT, N>& _ref)
{
  return epsilon(length(_ref));
}

template <typename ValueT1, typename ValueT2>
bool zero(const ValueT1& _val, const ValueT2& _ref)
{
  return std::abs(_val) < epsilon(_ref);
}


template <typename ValueT1, typename ValueT2>
bool zero_sq(const ValueT1& _val, const ValueT2& _ref)
{
  return std::abs(_val) < epsilon_sq(_ref);
}

}