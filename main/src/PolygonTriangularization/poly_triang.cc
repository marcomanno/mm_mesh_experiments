#include "poly_triang.hh"
#include "Geo/area.hh"
#include "Geo/entity.hh"
#include "Geo/plane_fitting.hh"
#include "Geo/linear_system.hh"
#include "Geo/point_in_polygon.hh"
#include "Geo/tolerance.hh"
#include "Utils/circular.hh"
#include "Utils/statistics.hh"
#include <Utils/error_handling.hh>
#include <numeric>

struct PolygonTriangulation : public IPolygonTriangulation
{
  virtual void add(const std::vector<Geo::Vector3>& _plgn) override;
  virtual const std::vector<std::array<size_t, 3>>& triangles() override
  {
    compute();
    return sol_.tris_;
  }

  const std::vector<Geo::Vector3>& polygon() override
  {
    compute();
    return loops_[0];
  }

  virtual double area() override
  {
    compute();
    return sol_.area_;
  }

private:
  struct Solution
  {
    void compute(const std::vector<Geo::Vector3>& _pos,
      std::vector<size_t>& _indcs,
      const double _tols,
      Geo::Vector<3>& _norm);
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
  };

  void compute();

  typedef std::vector<Geo::Vector3> Polygon;
  typedef std::vector<Polygon> PolygonVector;
  PolygonVector loops_;
  Solution sol_;
};

std::shared_ptr<IPolygonTriangulation> IPolygonTriangulation::make()
{
  return std::make_shared<PolygonTriangulation>();
}

void PolygonTriangulation::add(
  const std::vector<Geo::Vector3>& _plgn)
{
  loops_.push_back(_plgn);
  sol_.area_ = 0;
}

void PolygonTriangulation::compute()
{
  if (sol_.area_ > 0 || loops_.empty())
    return; // Triangulation already computed.

  auto pl_fit = Geo::IPlaneFit::make();
  auto pts_nmbr = loops_[0].size();
  for (int i = 0; ++i < loops_.size(); )
    pts_nmbr += loops_[i].size();
  pl_fit->init(pts_nmbr);
  for (const auto& loop : loops_)
    for (const auto& pt : loop)
      pl_fit->add_point(pt);
  Geo::Vector<3> centr, norm;
  pl_fit->compute(centr, norm);

  Utils::StatisticsT<double> tol_max;
  for (const auto& loop : loops_)
    for (const auto& pt : loop)
      tol_max.add(Geo::epsilon_sq(pt - centr));
  const auto tol = tol_max.max() * 10;

  if (loops_.size() > 1)
  {
    // Put the outer loop at the begin of the list.
    auto pt = loops_[0][0];
    for (auto loop_it = std::next(loops_.begin());
      loop_it != loops_.end(); 
      ++loop_it)
    {
      auto where = 
        Geo::PointInPolygon::classify(*loop_it, pt, tol, &norm);
      if (where == Geo::PointInPolygon::Inside)
      {
        std::swap(loops_.front(), *loop_it);
        break;
      }
    }
    while (loops_.size() > 1)
    {
      Utils::StatisticsT<double> stats;
      auto pt0 = loops_[0].back();
      std::tuple<Polygon::iterator,       // Near segment end on outer loop.
                 PolygonVector::iterator, // Nearest internal poop
                 Polygon::iterator,       // Nearest vertex on the intrnal loop
                 double>                  // Projection parameter
        conn_info, best_conn_info;
      for (std::get<0>(conn_info) = loops_[0].begin();
        std::get<0>(conn_info) != loops_[0].end(); 
        pt0 = *(std::get<0>(conn_info)++))
      {
        Geo::Segment seg = { pt0, *std::get<0>(conn_info) };
        for (std::get<1>(conn_info) = std::next(loops_.begin());
          std::get<1>(conn_info) != loops_.end();
          ++std::get<1>(conn_info))
        {
          for (std::get<2>(conn_info) = std::get<1>(conn_info)->begin();
            std::get<2>(conn_info) != std::get<1>(conn_info)->end();
            ++std::get<2>(conn_info))
          {
            double dist_sq;
            if (!Geo::closest_point(seg, *std::get<2>(conn_info),
              nullptr, &std::get<3>(conn_info), &dist_sq))
            {
              continue;
            }
            if (stats.add(dist_sq) & stats.Smallest)
              best_conn_info = conn_info;
          }
        }
      }
      if (std::get<3>(best_conn_info) < 0.5)
      {
        if (std::get<0>(best_conn_info) == loops_[0].begin())
          std::get<0>(best_conn_info) = loops_[0].end();
        --std::get<0>(best_conn_info);
      }
      std::rotate(std::get<1>(best_conn_info)->begin(),
        std::get<2>(best_conn_info), std::get<1>(best_conn_info)->end());
      std::get<1>(best_conn_info)->push_back(
        std::get<1>(best_conn_info)->front());
      std::get<1>(best_conn_info)->insert(
        std::get<1>(best_conn_info)->begin(),
        *std::get<0>(best_conn_info));
      loops_[0].insert(std::next(std::get<0>(best_conn_info)),
        std::get<1>(best_conn_info)->crbegin(),
        std::get<1>(best_conn_info)->crend());
      loops_.erase(std::get<1>(best_conn_info));
    }
  }
  // creates the indexvector removing duplicates.
  std::vector<rsize_t> indcs;
  indcs.reserve(loops_[0].size());
  for (size_t i = 0, j; i < loops_[0].size(); ++i)
  {
    for (j = 0; j < i; ++j)
      if (loops_[0][i] == loops_[0][j])
      {
        loops_[0].erase(loops_[0].begin() + (i--));
        break;
      }
    indcs.push_back(j);
  }
  auto tmp_loop = loops_[0];
  for (auto& pt : tmp_loop) pt -= centr;
  sol_.compute(tmp_loop, indcs, tol, norm);
}

void PolygonTriangulation::Solution::compute(
  const std::vector<Geo::Vector3>& _pts,
  std::vector<size_t>& _indcs,
  const double _tol,
  Geo::Vector<3>& _norm)
{
  auto valid_triangle = [&_indcs, &_pts, &_tol, &_norm](const size_t _i)
  {
    auto next = _i;
    auto idx = Utils::decrease(_i, _indcs.size());
    auto prev = Utils::decrease(idx, _indcs.size());
    std::vector<Geo::Vector3> tmp_poly(3);
    tmp_poly[0] = _pts[_indcs[prev]];
    tmp_poly[1] = _pts[_indcs[idx]];
    tmp_poly[2] = _pts[_indcs[next]];
    bool verify = true;
    for (size_t i = Utils::increase(_i, _indcs.size()); i != prev;
      i = Utils::increase(i, _indcs.size()))
    {
      bool joint_point = _indcs[i] == _indcs[next] || _indcs[i] == _indcs[prev];
      if (verify)
      {
        auto where = Geo::PointInPolygon::classify(
          tmp_poly, _pts[_indcs[i]], _tol, &_norm);
        if (where != Geo::PointInPolygon::Outside)
          return false;
      }
      verify = joint_point;
    }
    tmp_poly.clear();
    for (const auto& ind : _indcs)
      tmp_poly.push_back(_pts[ind]);
    auto pt_in = (tmp_poly[prev] + tmp_poly[next]) * 0.5;
    auto where = Geo::PointInPolygon::classify(tmp_poly, pt_in, _tol, &_norm);
    if (where != Geo::PointInPolygon::Inside)
      return false;
    for (auto& pt : tmp_poly)
      pt -= (pt * _norm) * _norm;
    auto j = tmp_poly.size() - 1;
    Geo::Segment seg = { tmp_poly[prev], tmp_poly[next] };
    for (size_t i = 0; i < tmp_poly.size(); j = i++)
    {
      if (_indcs[i] == _indcs[next] || _indcs[i] == _indcs[prev])
        continue;
      double dist_sq = 0;
      double tol_sq = std::max(Geo::epsilon_sq(seg[0]), Geo::epsilon_sq(seg[1]));
      if (Geo::closest_point(seg, tmp_poly[i], nullptr, nullptr, &dist_sq) &&
        dist_sq <= tol_sq)
        return false;
      if (_indcs[j] == _indcs[next] || _indcs[j] == _indcs[prev])
        continue;
      Geo::Segment seg1 = { tmp_poly[i], tmp_poly[j] };
      tol_sq = std::max({ tol_sq,
        Geo::epsilon_sq(seg1[0]), Geo::epsilon_sq(seg1[1]) });
      if (Geo::closest_point(seg, seg1,
        nullptr, nullptr, &dist_sq) && dist_sq <= tol_sq)
      {
        return false;
      }
    }
    return true;
  };

  while (_indcs.size() > 3)
  {
    Geo::Vector3 vects[2];
    size_t inds[3] = { *(_indcs.end() - 2), _indcs.back(), 0 };
    vects[0] = _pts[inds[0]] - _pts[inds[1]];
    Utils::StatisticsT<double> min_ang;
    for (size_t i = 0; i < _indcs.size(); ++i)
    {
      inds[2] = _indcs[i];
      vects[1] = _pts[inds[2]] - _pts[inds[1]];
      if (valid_triangle(i))
      {
        auto angl = Geo::angle(vects[0], vects[1]);
        min_ang.add(angl, i);
      }
      inds[0] = inds[1];
      inds[1] = inds[2];
      vects[0] = -vects[1];
    }
    if (min_ang.count() == 0)
    {
      THROW("No good triangle found.");
    }
    std::array<size_t, 3> tri;
    tri[2] = min_ang.min_idx();
    tri[1] = Utils::decrease(tri[2], _indcs.size());
    auto to_rem = tri[1];
    if (min_ang.min() > 0)
    {
      tri[0] = Utils::decrease(tri[1], _indcs.size());
      for (auto& pt_ind : tri) pt_ind = _indcs[pt_ind];
      tris_.push_back(tri);
    }
    _indcs.erase(_indcs.begin() + to_rem);
  }
  if (_indcs.size() == 3)
  {
    std::array<size_t, 3> tri = { 0, 1, 2 };
    for (auto& pt_ind : tri) pt_ind = _indcs[pt_ind];
    tris_.push_back(tri);
  }
  area_ = 0.;
  for (const auto& tri : tris_)
    area_ += Geo::area(_pts[tri[0]], _pts[tri[1]], _pts[tri[2]]);
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

bool PolygonTriangulation::Solution::find_concave(
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

bool PolygonTriangulation::Solution::contain_concave(
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
