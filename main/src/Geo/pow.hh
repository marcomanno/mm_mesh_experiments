#pragma once

#include <type_traits>

namespace Geo
{
template <typename numeric_type> class Pow
{
  template <bool _oddT> static void multply_if_odd(numeric_type&, const numeric_type&) {}
  template <> static void multply_if_odd<true>(numeric_type & _res, const numeric_type & v)
  {
    _res *= v;
  }
  template <int N> static numeric_type calc(const numeric_type & v)
  {
    numeric_type a = calc<N / 2>(v);
    a *= a;
    multply_if_odd< bool((N % 2) > 0) > (a, v);
    return a;
  }
  template <> static numeric_type calc<0>(const numeric_type & v)
  {
    return static_cast<numeric_type>(1);
  }
  template <> static numeric_type calc<1>(const numeric_type & v)
  {
    return v;
  }

  template <int N> static numeric_type pow_private(const numeric_type & v,
      std::integral_constant<bool, true>)
  {
    return calc<N>(v);
  }
  template <int N> static numeric_type pow_private(const numeric_type & v,
      std::integral_constant<bool, false>)
  {
    return static_cast<numeric_type>(1) / calc<-N>(v);
  }
public:
  template <int N> static numeric_type to(const numeric_type & v)
  {
    return pow_private<N>(v, std::integral_constant<bool, bool(N>=0)>());
  }
};

template <class num> inline num sq(const num v)
{
  return Pow<num>::template to<2>(v);
}
} // namespace Geo

