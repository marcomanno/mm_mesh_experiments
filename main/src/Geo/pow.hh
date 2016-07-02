#pragma once

#include <type_traits>

namespace Geo
{
template <typename numeric_type> class gk_pow
{
  template <int N> static numeric_type calc(const numeric_type & v)
  {
    if (N == 0)
      return static_cast<numeric_type>(1);
    if (N == 1)
      return v;
    numeric_type a = calc<N/2>(v);
    a *= a;
    if(N & 1)
      a *= v;
    return a;
  }
  template <int N> static numeric_type geo_pow_priv(const numeric_type & v,
      std::integral_constant<bool, true>)
  {
    return calc<N>(v);
  }
  template <int N> static numeric_type geo_pow_priv(const numeric_type & v,
      std::integral_constant<bool, false>)
  {
    return static_cast<numeric_type>(1) / calc<-N>(v);
  }
public:
  template <int N> static numeric_type to(const numeric_type & v)
  {
    return geo_pow_priv<N>(v, std::integral_constant<bool, bool(N>=0)>());
  }
};

template <class num> inline num gk_sq(const num v)
{
  return gk_pow<num>::template to<2>(v);
}
} // namespace Geo

