
#pragma once

#include "iterate.hh"

namespace Geo
{

template <size_t kT> constexpr size_t comb(const size_t _n)
{
  return (comb<kT - 1>(_n - 1) * _n) / kT;
}

template <> constexpr size_t comb<0>(const size_t) { return 1; }

inline size_t comb(size_t _n, size_t _k)
{
  if (_k > _n)
    return 1;
  size_t res = 1;
  for (size_t i = 1; i <= _k; ++i)
  {
    res *= _n--;
    res /= i;
  }
  return res;
}

} // namespace Geo
