#include "poly_triang.hh"
#include "statistics.hh"
#include "linear_system.hh"
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
    auto orig_ind = indcs_.back();
    vects[0] = _pts[*(indcs_.end() - 2)] - _pts[orig_ind];
    Geo::StatisticsT<double> min_ang;
    for (const auto& ind : indcs_)
    {
      vects[1] = _pts[ind] - _pts[orig_ind];
      auto angl = Geo::angle(vects[0], vects[1]);
      if (concave(orig_ind))
        angl = 2 * M_PI - angl;
      min_ang.add(angl);
      orig_ind = ind;
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
size_t inside_triangle(const Geo::Vector3& _pt,
  const Geo::Vector3& _vrt0, 
  const Geo::Vector3& _vrt1, 
  const Geo::Vector3& _vrt2
  )
{
  const Geo::Vector3 pts[2] = {_vrt1 - _vrt0, _vrt2 - _vrt0}, 
    b(_pt - _vrt0);

  double A[2][2], B[2], X[2];
  for (int i = 0; i < 2; ++i)
  {
    for (int j = i; j < 2; ++j)
      A[i][j] = A[j][i] = pts[i] * pts[j];
    B[i] = pts[i] * b;
  }
  if (!Geo::solve_2x2(A, X, B))
    return 0;

  size_t result = 0;
  if (X[0] >= 0 && X[1] >= 0 && (X[0] + X[1]) <= 1)
  {
    ++result;
    if (X[0] > 0 && X[1] > 0 && (X[0] + X[1]) < 1)
      ++result;
  }
  return result;
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
    size_t inside = 0;
    for (const auto& tri : tris_)
    {
      if (std::find(tri.begin(), tri.end(), i) != tri.end())
        continue;
      inside += inside_triangle(
        _pts[i], _pts[tri[0]], _pts[tri[1]], _pts[tri[2]]);
      if (inside > 1)
      {
        _concav[i] = !_concav[i];
        achange = true;
        break;
      }
    }
  }
  return achange;
}