#include "poly_triang.hh"
#include "circular.hh"
#include "Utils/statistics.hh"
#include "Geo/area.hh"
#include "Geo/entity.hh"
#include "Geo/linear_system.hh"
#include "Geo/point_in_polygon.hh"
#include "Geo/tolerance.hh"
#include <numeric>

struct PolygonFilImpl : public PolygonFil
{
  void init(const std::vector<Geo::Vector3>& _plgn);
  virtual const std::vector<std::array<size_t, 3>>& triangles() const
  {
    return sol_.tris_;
  }
  virtual const std::vector<Geo::Vector3>& positions() const
  {
    return pts_;
  }
  virtual double area_compute() const
  {
    return sol_.area_;
  }

private:
  struct Solution
  {
    void compute(const std::vector<Geo::Vector3>& _pos);
    bool concave(size_t _i) const
    {
      return _i < concav_.size() && concav_[_i];
    }

    bool contain_concave(size_t _inds[3], Geo::Vector3 _vects[2],
      const std::vector<Geo::Vector3>& _pts) const;

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
  if (sol_.find_concave(_plgn, sol.concav_))
  {
    sol.compute(_plgn);
    if (sol.area_ < sol_.area_)
      sol_ = sol;
  }
}

namespace {
//!Find if the triangle is completely insidethe polygon
bool valid_triangle(const size_t _i,
  const std::vector<size_t>& _indcs, 
  const std::vector<Geo::Vector3>&  _pts,
  const double _tol)
{
  auto prev = Circular::decrease(_i, _indcs.size());
  auto next = Circular::increase(_i, _indcs.size());
  std::vector<Geo::Vector3> tmp_poly;
  for (const auto& ind : _indcs)
    tmp_poly.push_back(_pts[ind]);
  auto pt_in = (tmp_poly[prev] + tmp_poly[next]) * 0.5;
  auto where = Geo::PointInPolygon::classify(tmp_poly, pt_in, _tol);
  if (where != Geo::PointInPolygon::Inside)
    return false;
  auto j = tmp_poly.size() - 1;
  Geo::Segment segs[2] = {
    Geo::Segment({tmp_poly[prev], tmp_poly[_i]}),
    Geo::Segment({tmp_poly[next], tmp_poly[_i]}) };
  for (size_t i = 0; i < tmp_poly.size(); j = i++)
  {
    if (i == next || i == prev || j == next || j == prev)
      continue;
    Geo::Segment pol_seg = {tmp_poly[i], tmp_poly[j]};
    for (const auto& seg : segs)
    {
      if (Geo::closest_point(seg, pol_seg))
        return false;
    }
  }
  return true;
}
};

void PolygonFilImpl::Solution::compute(
  const std::vector<Geo::Vector3>& _pts)
{
  indcs_.resize(_pts.size());
  std::iota(indcs_.begin(), indcs_.end(), 0);
  Utils::StatisticsT<double> tol_max;
  for (const auto& pt : _pts)
    tol_max.add(Geo::epsilon(pt));
  auto tol = tol_max.max() * 10;
  for (;;)
  {
    if (indcs_.size() < 3)
      break;
    Geo::Vector3 vects[2];
    size_t inds[3] = { *(indcs_.end() - 2), indcs_.back(), 0 };
    vects[0] = _pts[inds[0]] - _pts[inds[1]];
    Utils::StatisticsT<double> min_ang;
    for (size_t i = 0; i < indcs_.size(); ++i)
    {
      inds[2] = indcs_[i];
      if (!concave(inds[1]))
      {
        vects[1] = _pts[inds[2]] - _pts[inds[1]];
        if (valid_triangle(i, indcs_, _pts, tol))
        // if (!contain_concave(inds, vects, _pts))
        {
          auto angl = Geo::angle(vects[0], vects[1]);
          min_ang.add(angl, i);
        }
      }
      inds[0] = inds[1];
      inds[1] = inds[2];
      vects[0] = -vects[1];
    }
    auto idx = min_ang.min_idx();
    /*
    auto decrease = [this](size_t& _idx)
    {
      if (_idx == 0) _idx = indcs_.size();
      return --_idx;
    };*/
    std::array<size_t, 3> tri;
    tri[2] = indcs_[idx];
    tri[1] = indcs_[idx = Circular::decrease(idx, indcs_.size())];
    indcs_.erase(indcs_.begin() + idx);
    tri[0] = indcs_[idx = Circular::decrease(idx, indcs_.size())];

    if (min_ang.min() > 0)
      tris_.push_back(tri);
  }
  area_ = 0.;
  for (const auto& tri : tris_)
    area_ += Geo::area_compute(_pts[tri[0]], _pts[tri[1]], _pts[tri[2]]);
}

namespace {

// return 0 - outside, 1 - on boundary, 2 - inside
size_t inside_triangle(
  const Geo::Vector3 _vert[2],
  const Geo::Vector3& _test_pt)
{
  double A[2][2], B[2], X[2];
  for (int i = 0; i < 2; ++i)
  {
    for (int j = i; j < 2; ++j)
      A[i][j] = A[j][i] = _vert[i] * _vert[j];
    B[i] = _vert[i] * _test_pt;
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

size_t inside_triangle(const Geo::Vector3& _pt,
  const Geo::Vector3& _vrt0, 
  const Geo::Vector3& _vrt1, 
  const Geo::Vector3& _vrt2
  )
{
  const Geo::Vector3 verts[2] = { _vrt1 - _vrt0, _vrt2 - _vrt0 };
  return inside_triangle(verts, _pt - _vrt0);
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

bool PolygonFilImpl::Solution::contain_concave(
  size_t _inds[3], Geo::Vector3 _vects[2],
  const std::vector<Geo::Vector3>& _pts) const
{
  for (auto i = 0; i < concav_.size(); ++i)
  {
    if (i == _inds[0] || i == _inds[2] || !concav_[i])
      continue;
    if (inside_triangle(_vects, _pts[i] - _pts[_inds[1]]))
      return true;
  }
  return false;
}
