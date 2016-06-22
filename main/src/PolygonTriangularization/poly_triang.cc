#include "poly_triang.hh"
#include "statistics.hh"
#include <numeric>

struct PolygonFilImpl : public PolygonFil
{
  void init(const std::vector<Geo::Vector3>& _plgn);
  const std::vector<std::array<size_t, 3>>& triangles() const
  {
    return sol_.tris_;
  }
  const std::vector<Geo::Vector3>& positions() const
  {
    return pts_;
  }

private:
  struct Solution
  {
    void compute(const std::vector<Geo::Vector3>& _pos);
    bool concave(size_t _i) const
    {
      return _i < concav_.size() && concav_[_i];
    }
    bool find_concave(const std::vector<Geo::Vector3>& _pts,
      std::vector<bool>& _concav) const;

    std::vector<std::array<size_t, 3>> tris_;
    double area_ = 0;
    std::vector<bool> concav_;
    std::vector<size_t> indcs_;
  };

  std::vector<Geo::Vector3> pts_;
  Solution sol_;
};

std::shared_ptr<PolygonFil> PolygonFil::make()
{
  return std::make_shared<PolygonFilImpl>();
}


void PolygonFilImpl::init(const std::vector<Geo::Vector3>& _plgn)
{
  pts_ = _plgn;
  sol_.compute(_plgn);
  Solution sol;
  sol_.find_concave(_plgn, sol.concav_);
  sol.compute(_plgn);
  if (sol.area_ < sol_.area_)
    sol_ = sol;
}

void PolygonFilImpl::Solution::compute(
  const std::vector<Geo::Vector3>& _pts)
{
  indcs_.resize(_pts.size());
  std::iota(indcs_.begin(), indcs_.end(), 0);
  for (;;)
  {
    if (indcs_.size() < 3)
      break;
    Geo::Vector3 vects[2];
    auto last = std::prev(indcs_.end());
    auto orig = _pts[*last];
    vects[0] = _pts[*std::prev(last)] - orig;

    Geo::StatisticsT<double> min_ang;

    for (const auto& ind : indcs_)
    {
      vects[1] = _pts[ind] - orig;
      auto angl = Geo::angle(vects[0], vects[1]);
      if (concave(ind))
        angl = M_PI - angl;
      min_ang.add(Geo::angle(vects[0], vects[1]));
      orig = _pts[ind];
      vects[0] = -vects[1];
    }
    auto idx = min_ang.min_idx();
    auto decrease = [this](size_t& _idx)
    {
      if (_idx == 0) _idx = indcs_.size();
      return --_idx;
    };
    std::array<size_t, 3> tri;
    tri[2] = indcs_[idx];
    tri[1] = indcs_[decrease(idx)];
    indcs_.erase(indcs_.begin() + idx);
    tri[0] = indcs_[decrease(idx)];

    if (min_ang.min() > 0)
      tris_.push_back(tri);
  }
  area_ = 0;
  for (const auto& tri : tris_)
  {
    const auto v0 = _pts[tri[1]] - _pts[tri[0]];
    const auto v1 = _pts[tri[2]] - _pts[tri[0]];
    area_ += Geo::length(v0 % v1);
  }
  area_ /= 2;
}
namespace {
bool inside_triangle(const Geo::Vector3& _pt,
  const Geo::Vector3& _pt0,
  const Geo::Vector3& _pt1,
  const Geo::Vector3& _pt2)
{
  auto v0 = _pt1 - _pt0;
  auto v1 = _pt2 - _pt0;
  auto x1 = Geo::length(v0);
  auto n = v0 % v1;
  n /= Geo::length(n);
  auto w = _pt - v0;
  w -= (w * n) * n;
  auto vn0 = v0 % n;
  auto ptx = w * v0 / x1;
  auto pty = w * vn0 / x1;
  auto x2 = v1 * v0 / x1;
  auto y2 = v1 * vn0 / x1;
  if (y2 < 0)
  {
    y2 = -y2;
    pty = -pty;
  }
  if (pty < 0 || pty >= y2)
    return false;
  auto rat = pty / y2;
  int inters_nmbr = 0;
  if (x2 * rat <= ptx)
    return false;
  return (x1 - rat * (x2 - x1)) > ptx;

}

}

bool PolygonFilImpl::Solution::find_concave(
  const std::vector<Geo::Vector3>& _pts,
  std::vector<bool>& _concav) const
{
  bool achange = false;
  _concav.resize(_pts.size());
  for (size_t i = 0; i < _pts.size(); ++i)
  {
    _concav[i] = concave(i);
    for (const auto& tri : tris_)
    {
      if (std::find(tri.begin(), tri.end(), i) == tri.end())
        continue;
      if (inside_triangle(_pts[i], _pts[tri[0]], _pts[tri[1]], _pts[tri[2]]))
      {
        _concav[i] = !_concav[i];
        achange = true;
        break;
      }
    }
  }
  return achange;
}