#pragma once

#include <array>
#include <limits>

namespace Geo {

template <typename ValueT>
ValueT epsilon(const ValueT& _ref)
{
  return (std::abs(_ref) + 1) * sqrt(std::numeric_limits<ValueT>::epsilon() * 10);
}

template <typename ValueT, size_t N>
ValueT epsilon(const std::array<ValueT, N>& _ref)
{
  ValueT dot = 0;
  for (const auto& val : _ref) dot += val * val;
  return epsilon(sqrt(dot));
}

template <typename ValueT1, typename ValueT2>
bool zero(const ValueT1& _val, const ValueT2& _ref)
{
  return std::abs(_val) < epsilon(_ref);
}

template <typename ValueT>
ValueT epsilon_sq(const ValueT& _ref)
{
  return (std::abs(_ref) + 1) * std::numeric_limits<ValueT>::epsilon() * 10;
}

template <typename ValueT, size_t N>
ValueT epsilon_sq(const std::array<ValueT, N>& _ref)
{
  ValueT dot = 0;
  for (const auto& val : _ref) dot += val * val;
  return epsilon_sq(dot);
}

template <typename ValueT1, typename ValueT2>
bool zero_sq(const ValueT1& _val, const ValueT2& _ref)
{
  return std::abs(_val) < epsilon_sq(_ref);
}

}