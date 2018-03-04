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

//#define DEBUG_PolygonTriangularization
#include "Import/import.hh"

namespace Geo
{

struct PolygonTriangulation : public IPolygonTriangulation
{
  virtual void add(const std::vector<Geo::VectorD3>& _plgn) override;
  virtual const std::vector<std::array<size_t, 3>>& triangles() override
  {
    compute();
    return sol_.tris_;
  }

  const std::vector<Geo::VectorD3>& polygon() override
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
    void compute(const std::vector<Geo::VectorD3>& _pos,
                 std::vector<size_t>& _indcs,
                 const double _tols,
                 Geo::VectorD<3>& _norm);
    bool concave(size_t _i) const
    {
      return _i < concav_.size() && concav_[_i];
    }

    bool contain_concave(size_t _inds[3], Geo::VectorD3 _vects[2],
                         const std::vector<Geo::VectorD3>& _pts) const;

    bool find_concave(const std::vector<Geo::VectorD3>& _pts,
                      std::vector<bool>& _concav) const;

    std::vector<std::array<size_t, 3>> tris_;
    double area_ = 0;
    std::vector<bool> concav_;
  };

  void compute();

  typedef std::vector<Geo::VectorD3> Polygon;
  typedef std::vector<Polygon> PolygonVector;
  PolygonVector loops_;
  Solution sol_;
};

std::shared_ptr<IPolygonTriangulation> IPolygonTriangulation::make()
{
  return std::make_shared<PolygonTriangulation>();
}

void PolygonTriangulation::add(
  const std::vector<Geo::VectorD3>& _plgn)
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
  Geo::VectorD<3> centr, norm;
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
      struct ConnInfo
      {
        Polygon::iterator near_bnd_v, near_isl_v;
        PolygonVector::iterator near_island;
        double near_par = 0;
      } ci, bci; // connection info and best connection info

      for (ci.near_bnd_v = loops_[0].begin();
           ci.near_bnd_v != loops_[0].end();
           pt0 = *(ci.near_bnd_v++))
      {
        Geo::Segment seg = { pt0, *ci.near_bnd_v };
        for (ci.near_island = std::next(loops_.begin());
             ci.near_island != loops_.end();
             ++ci.near_island)
        {
          for (ci.near_isl_v = ci.near_island->begin();
               ci.near_isl_v != ci.near_island->end();
               ++ci.near_isl_v)
          {
            double dist_sq;
            if (!Geo::closest_point(seg, *ci.near_isl_v,
                                    nullptr, &ci.near_par, &dist_sq))
            {
              continue;
            }
            if (stats.add(dist_sq) & stats.Smallest)
              bci = ci;
          }
        }
      }
      if (bci.near_par < 0.5)
      {
        if (bci.near_bnd_v == loops_[0].begin())
          bci.near_bnd_v = loops_[0].end();
        --bci.near_bnd_v;
      }
      std::rotate(bci.near_island->begin(), bci.near_isl_v, bci.near_island->end());
      bci.near_island->push_back(bci.near_island->front());
      bci.near_island->push_back(*bci.near_bnd_v);
      loops_[0].insert(std::next(bci.near_bnd_v),
                       bci.near_island->cbegin(),
                       bci.near_island->cend());
      loops_.erase(bci.near_island);
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
  sol_.compute(loops_[0], indcs, tol, norm);
}

void PolygonTriangulation::Solution::compute(
  const std::vector<Geo::VectorD3>& _pts,
  std::vector<size_t>& _indcs,
  const double,
  Geo::VectorD<3>&)
{
  auto valid_triangle = [&_indcs, &_pts](const size_t _i,
                                         const std::vector<Geo::VectorD3>& proj_poly,
                                         const Geo::VectorD3& _norm,
                                         const double& _tol)
  {
    auto next = _i;
    auto idx = Utils::decrease(_i, _indcs.size());
    auto prev = Utils::decrease(idx, _indcs.size());

    std::vector<Geo::VectorD3> tmp_poly(3);
    tmp_poly[0] = _pts[_indcs[prev]];
    tmp_poly[1] = _pts[_indcs[idx]];
    tmp_poly[2] = _pts[_indcs[next]];
    // Check that one other vertex is not isnide thetriangle.
    // Just to be shure that the rest of the chain is not completely
    // inside the new triangle.
    for (auto i : _indcs)
    {
      if (i == _indcs[next] || i == _indcs[prev] || i == _indcs[idx])
        continue;
      auto where = Geo::PointInPolygon::classify(tmp_poly, _pts[i], _tol, &_norm);
      if (where != Geo::PointInPolygon::Outside)
        return false;
    }
    for (auto frac : { 0.5, 0.25, 0.75 })
    {
      auto pt_in = proj_poly[prev] * frac + proj_poly[next] * (1 - frac);
      auto where = Geo::PointInPolygon::classify(
        proj_poly, pt_in, Geo::epsilon(pt_in), &_norm);
      if (where == Geo::PointInPolygon::Outside)
        return false;
      if (where == Geo::PointInPolygon::Inside)
        break;
    }
    auto j = proj_poly.size() - 1;
    Geo::Segment seg = { proj_poly[prev], proj_poly[next] };
    for (size_t i = 0; i < proj_poly.size(); j = i++)
    {
      if (_indcs[i] == _indcs[next] || _indcs[i] == _indcs[prev])
        continue;
      double dist_sq = 0;
      //const auto prec = std::numeric_limits<double>::epsilon() * 100;
      //double tol_sq = prec * std::max(Geo::length_square(seg[0]), Geo::length_square(seg[1]));
      double tol_sq = std::max(Geo::epsilon_sq(seg[0]), Geo::epsilon_sq(seg[1]));
      if (Geo::closest_point(seg, proj_poly[i], nullptr, nullptr, &dist_sq) &&
          dist_sq <= tol_sq)
        return false;
      if (_indcs[j] == _indcs[next] || _indcs[j] == _indcs[prev])
        continue;
      Geo::Segment seg1 = { proj_poly[i], proj_poly[j] };
      //tol_sq = std::max(tol_sq, prec * std::max(Geo::length_square(seg1[0]), Geo::length_square(seg1[1])));
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
#ifdef DEBUG_PolygonTriangularization
    std::string flnm("debug_poly_");
    flnm += std::to_string(_indcs.size()) + ".obj";
    IO::save_obj(flnm.c_str(), _pts, &_indcs);
#endif

    Geo::VectorD3 vects[2], centre;
    size_t inds[3] = { *(_indcs.end() - 2), _indcs.back(), 0 };
    vects[0] = _pts[inds[0]] - _pts[inds[1]];
    auto norm = Geo::point_polygon_normal(_pts.begin(), _pts.end(), &centre);

    std::vector<Geo::VectorD3> proj_poly;
    Utils::StatisticsT<double> tol_max;
    for (auto ii : _indcs)
    {
      auto pt = _pts[ii] - centre;
      pt -= (pt * norm) * norm;
      proj_poly.push_back(pt);
      tol_max.add(Geo::epsilon_sq(pt));
    }
    auto tol = sqrt(9 * tol_max.max());
    std::vector<double> angles;
    const auto invalid_double = std::numeric_limits<double>::max();

    for (size_t i = 0; i < _indcs.size(); ++i)
    {
      inds[2] = _indcs[i];
      vects[1] = _pts[inds[2]] - _pts[inds[1]];
      if (inds[0] != inds[2] && valid_triangle(i, proj_poly, norm, tol))
        angles.push_back(Geo::signed_angle(vects[1], vects[0], norm));
      else
        angles.push_back(invalid_double);
      inds[0] = inds[1];
      inds[1] = inds[2];
      vects[0] = -vects[1];
    }
    for (auto& ang : angles)
      if (ang < 0)
        ang += M_PI;
    std::vector<double> scores(angles);
    for (size_t i = 0; i < scores.size(); ++i)
    {
      if (angles[i] == invalid_double)
      {
        scores[Utils::decrease(i, scores.size())] -= M_PI;
        scores[Utils::increase(i, scores.size())] -= M_PI;
      }
    }
    Utils::StatisticsT<double> min_ang;
    for (size_t i = 0; i < scores.size(); ++i)
      min_ang.add(scores[i], i);
    if (min_ang.min() == invalid_double)
    {
      IO::save_obj("No_good_triangle_found", _pts, &_indcs);
      THROW("No good triangle found.");
    }
    std::array<size_t, 3> tri;
    tri[2] = min_ang.min_idx();
    tri[1] = Utils::decrease(tri[2], _indcs.size());
    auto to_rem = tri[1];
    tri[0] = Utils::decrease(tri[1], _indcs.size());
    for (auto& pt_ind : tri) pt_ind = _indcs[pt_ind];
    tris_.push_back(tri);
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
  const Geo::VectorD3 _vert[2],
  const Geo::VectorD3& _test_pt)
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

size_t inside_triangle(const Geo::VectorD3& _pt,
                       const Geo::VectorD3& _vrt0,
                       const Geo::VectorD3& _vrt1,
                       const Geo::VectorD3& _vrt2
)
{
  const Geo::VectorD3 verts[2] = { _vrt1 - _vrt0, _vrt2 - _vrt0 };
  return inside_triangle(verts, _pt - _vrt0);
}

}

bool PolygonTriangulation::Solution::find_concave(
  const std::vector<Geo::VectorD3>& _pts,
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
  size_t _inds[3], Geo::VectorD3 _vects[2],
  const std::vector<Geo::VectorD3>& _pts) const
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

} // namespace Geo