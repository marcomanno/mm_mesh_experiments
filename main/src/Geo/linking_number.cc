
#include "linking_number.hh"

namespace Geo
{
namespace {
double contribution(const Geo::VectorD3 _p[2][2])
{
  Geo::VectorD3 T[2];
  Geo::VectorD2 len;
  for (auto i = 0; i < 2; ++i)
  {
    T[i] = _p[i][1] - _p[i][0];
    len[i] = Geo::length(T[i]);
    T[i] /= len[i];
  }
  auto dp = _p[0][0] - _p[1][0];
  const double SAMPLES = 8;
  Geo::VectorD2 dt = len / SAMPLES;
  Geo::VectorD2 u;
  //auto coeff = (T[0] % T[1]) * dp * dt[0] * dt[1];
  double integr_sum = 0;
  double coeff = (T[0] % T[1]) * dp * dt[0] * dt[1];
  for (u[0] = dt[0] / 2.; u[0] < len[0]; u[0] += dt[0])
    for (u[1] = dt[1] / 2.; u[1] < len[1]; u[1] += dt[1])
    {
      auto v = dp + T[0] * u[0] - T[1] * u[1];
      auto vlen_sq = Geo::length_square(v);
      integr_sum += coeff / (vlen_sq * sqrt(vlen_sq));
    }
  return integr_sum /* * coeff*/;
}

long double contribution1(const Geo::VectorD3 _p[2][2])
{
  Geo::VectorD3 T[2];
  for (auto i : { 0, 1 })
    T[i] = _p[i][1] - _p[i][0];
  auto dp = _p[0][0] - _p[1][0];
  const double SAMPLES = 4096;
  double dt = 1. / SAMPLES;
  Geo::VectorD2 u;
  long double integr_sum = 0;
  double coeff = (T[0] % T[1]) * dp * dt * dt;
  for (u[0] = dt / 2.; u[0] < 1.; u[0] += dt)
    for (u[1] = dt / 2.; u[1] < 1.; u[1] += dt)
    {
      auto v = dp + T[0] * u[0] - T[1] * u[1];
      long double vlen_sq = 0;
      for (auto c : v)
        vlen_sq += c * c;
      integr_sum += 1. / (vlen_sq * sqrt(vlen_sq));
    }
  return coeff * integr_sum;
}

} // namespace


int LinkingNumber::compute(
  const std::vector<Geo::VectorD3>& _loop0,
  const std::vector<Geo::VectorD3>& _loop1)
{
  if (_loop0.size() < 3 || _loop1.size() < 3)
    return 0;
  long double link_numb = 0;
  Geo::VectorD3 p[2][2];
  p[0][0] = _loop0.back();
  for (const auto& p1 : _loop0)
  {
    p[0][1] = p1;
    p[1][0] = _loop1.back();
    long double cc = 0;
    for (const auto& q1 : _loop1)
    {
      p[1][1] = q1;
      auto cc_l = contribution1(p);
      std::cout << " " << cc_l;
      cc += cc_l;
      p[1][0] = p[1][1];
    }
    std::cout << std::endl << cc << std::endl;
    link_numb += cc;
    p[0][0] = p[0][1];
  }
  link_numb /= 4 * M_PI;
  std::cout << "link_numb = " << link_numb << std::endl;
  return static_cast<int>(std::round(link_numb));
}

} // namespace Geo
