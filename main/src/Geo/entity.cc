
#include <entity.hh>
#include <vector.hh>
#include <pow.hh>
#include <linear_system.hh>
#include <Utils/statistics.hh>

#include <vector>

namespace Geo {

namespace {
double check_par(double _t) { return _t >= 0 && _t <= 1; }
double check_par(double _u, double _v)
{
  return _u >= 0 && _v >= 0 && (1 - _u - _v) >= 0;
}

}//namespace

Point evaluate(const Segment& _seg, double _t)
{
  return (1 - _t) * _seg[0] + _t * _seg[1];
}

Point evaluate(const Triangle& _tri, double _u, double _v)
{
  return _u * _tri[0] + _v * _tri[1] + (1 - _u - _v) * _tri[2];
}

bool closest_point(const Segment& _seg, const Point& _pt,
  Point* _clsst_pt, double* _t, double * _dist_sq)
{
  Point pt_diff[2] = { _pt - _seg[0], _pt - _seg[1] };
  double dists_sq[2] = { length_square(pt_diff[0]), length_square(pt_diff[1]) };
  size_t orig = dists_sq[0] < dists_sq[1] ? 0 : 1;
  auto p01 = (_seg[1 - orig] - _seg[orig]);
  double par = pt_diff[orig] * p01;
  if (par < 0) // Projection is outside the segment, closest point is the extreme.
  {
    if (_t != nullptr)
      *_t = double(orig);
    if (_dist_sq != nullptr)
      *_dist_sq = dists_sq[orig];
    if (_clsst_pt != nullptr)
      *_clsst_pt = _seg[orig];
  }
  else
  {
    if (_clsst_pt != nullptr || _dist_sq != nullptr)
    {
      auto clsst_pt = _seg[orig] + (par / length_square(p01)) * p01;
      if (_clsst_pt != nullptr)
        *_clsst_pt = clsst_pt;
      if (_dist_sq != nullptr)
        *_dist_sq = length_square(_pt - clsst_pt);
    }
    if (_t != nullptr)
    {
      *_t = (par / length_square(p01));
      if (orig > 0)
        *_t = 1 - *_t;
    }
  }
  return true;
}

bool closest_point(const Segment& _seg_a, const Segment& _seg_b,
  Point* _clsst_pt, double _t[2], double * _dist_sq)
{
  auto A = _seg_a[0] - _seg_b[0];
  auto b = _seg_a[1] - _seg_a[0];
  auto c = _seg_b[1] - _seg_b[0];
  auto discr = (b * b) * (c * c) - gk_sq(b * c);
  Utils::StatisticsT<double> len_stats;
  len_stats.add(length_square(_seg_a[1]));
  len_stats.add(length_square(_seg_a[1]));
  len_stats.add(length_square(_seg_b[0]));
  len_stats.add(length_square(_seg_b[1]));

  if (zero(discr, len_stats.max()))
    return false;
  double t[2];
  t[0] = ((A * c) * (b * c) - (A * b) * (c * c)) / discr;
  t[1] = ((A * c) * (b * b) - (A * b) * (b * c)) / discr;
  if (!check_par(t[0]) || !check_par(t[1]))
    return false;
  auto pt_a = evaluate(_seg_a, t[0]);
  auto pt_b = evaluate(_seg_b, t[1]);
  if (_clsst_pt != nullptr)
    *_clsst_pt = (pt_a + pt_b) / 2.;
  if (_dist_sq != nullptr)
    *_dist_sq = length_square(pt_a - pt_b);
  if (_t != nullptr)
  {
    _t[0] = t[0];
    _t[1] = t[1];
  }
  return true;
}

namespace {
struct PolygonalFace : public IPolygonalFace
{
  virtual bool triangle(size_t _idx, Triangle& _tri) const
  {
    if (_idx > tris_.size())
      return false;
    _tri = tris_[_idx];
    return true;
  }
  virtual size_t triangle_number() const { return tris_.size(); }

  virtual Point normal() const;

protected:
  virtual void add_point(const Point& _pt) { pts_.push_back(_pt); }
  virtual void finalize();

private:
  std::vector<Triangle> tris_;
  std::vector<Point> pts_;
};

void PolygonalFace::finalize()
{
  // This is not efficient at all. Cache the angle computed in previous iteration.
  while (pts_.size() > 3)
  {
    auto index = [this](size_t _i, size_t _back_off)
    {
      if (_i >= _back_off) return _i - _back_off;
      return pts_.size() - 1 - _back_off - _i;
    };
    Utils::StatisticsT<double> angl_stat;
    for (size_t i = 0; i < pts_.size(); ++i)
    {
      auto vp = pts_[index(i, 2)] - pts_[index(i, 1)];
      auto vn = pts_[index(i, 0)] - pts_[index(i, 1)];
      angl_stat.add(angle(vp, vn));
    }
    auto min_ind = angl_stat.min_idx();
    tris_.push_back({
      pts_[index(min_ind, 2)], 
      pts_[index(min_ind, 1)], 
      pts_[index(min_ind, 0)] });
    pts_.erase(pts_.begin() + index(min_ind, 1));
  }
}

Point PolygonalFace::normal() const
{
  Point normal{ 0,0,0 };
  for (auto& tri : tris_)
    normal += (tri[1] - tri[0]) % (tri[2] - tri[0]);
  auto len = length(normal);
  if (len > 0)
    normal /= len;
  return normal;
}

}//namespace

std::shared_ptr<IPolygonalFace> IPolygonalFace::make_me()
{
  return std::make_shared<PolygonalFace>();
}

// Finds u and v such that the distance between pt and
// _tri[0] * u + _tri[1] * v + _tri[2] * (1 - u - v)
// is minimal. u and v must be > 0 and u + v < 1.
// If the constraints are not satisfied, returns false.
bool closest_point(const Triangle& _tri, const Point& _pt,
  Point* _clsst_pt, double * _dist_sq)
{
  double A[2][2], B[2];
  auto v0 = _tri[0] - _tri[2];
  auto v1 = _tri[1] - _tri[2];
  auto dp = _pt - _tri[2];

  A[0][0] = length_square(v0);
  A[1][1] = length_square(v1);
  A[0][1] = A[1][0] = v0 * v1;

  B[0] = dp * v0;
  B[1] = dp * v1;

  double uv[2];
  if (!solve_2x2(A, uv, B))
    return false;
  if (!check_par(uv[0], uv[1]))
    return false;
  auto clsst_pt = _tri[2] + uv[0] * (_tri[0] - _tri[2]) + uv[1] * (_tri[1] - _tri[2]);
  if (_clsst_pt != nullptr)
    *_clsst_pt = clsst_pt;
  if (_dist_sq != nullptr)
    *_dist_sq = length(clsst_pt - _pt);
  return true;
}

bool closest_point(const IPolygonalFace& _face, const Point& _pt,
  Point* _clsst_pt, double * _dist_sq)
{
  Utils::StatisticsT<double> dist_stats;
  Triangle tri;
  Point clsst_pt;
  for (size_t i = 0; i < _face.triangle_number(); ++i)
  {
    if (!_face.triangle(i, tri))
      continue;
    double dist_sq = 0;
    if (!closest_point(tri, _pt, &clsst_pt, &dist_sq))
      continue;
    if (dist_stats.add(dist_sq) & dist_stats.Smallest && _clsst_pt != nullptr)
      *_clsst_pt = clsst_pt;
  }
  if (_dist_sq != nullptr)
    *_dist_sq = dist_stats.min();
  return true;
}

bool closest_point(const Triangle& _tri, const Segment& _seg,
  Point* _clsst_pt, double * _t, double * _dist_sq)
{
  auto a = _tri[0] - _seg[0];
  auto b = _tri[1] - _seg[0];
  auto c = _tri[2] - _seg[0];
  auto p = _seg[1] - _seg[0];

  // Minimize ||a * u + b * v + c * (1 - u - v) - p * t||^2 in u, v, t
  auto aa = a * a;
  auto ab = a * b;
  auto ac = a * c;
  auto bb = b * b;
  auto bc = b * c;
  auto cc = c * c;
  auto ap = a * p;
  auto bp = b * p;
  auto cp = c * p;

  // A[0] = d/du = 0, A[1] = d/dv= 0, A[2] = d/dt = 0
  double A[3][3];
  A[0][0] = A[1][1] = cc - 2 * ac;
  A[0][0] += aa;
  A[1][1] += bb;
  A[0][1] = A[1][0] = cc + ab - ac - bc;
  A[0][2] = A[1][2] = cp - ap - bp;
  A[2][0] = cp - ap;
  A[2][0] = cp - bp;
  A[2][2] = p * p;
  double B[3] = { cc - ac, cc - bc, cp };
  double uvt[3];
  if (!solve_3x3(A, uvt, B))
    return false;
  if (!check_par(uvt[0], uvt[1]) && !check_par(uvt[2]))
    return false;
  auto pt_seg = evaluate(_seg, uvt[2]);
  auto pt_tri = evaluate(_tri, uvt[0], uvt[1]);
  if (_clsst_pt!= nullptr)
    *_clsst_pt = (pt_seg + pt_tri) / 2.;
  if (_t != nullptr)
    *_t = uvt[2];
  if (_dist_sq != nullptr)
    *_dist_sq = length_square(pt_seg - pt_tri);
  return true;
}

bool closest_point(const IPolygonalFace& _face, const Segment& _pt,
  Point* _clsst_pt, double * _t, double * _dist_sq)
{
  Utils::StatisticsT<double> dist_stats;
  Triangle tri;
  Point clsst_pt;
  for (size_t i = 0; i < _face.triangle_number(); ++i)
  {
    if (!_face.triangle(i, tri))
      continue;
    double dist_sq = 0, t;
    if (!closest_point(tri, _pt, &clsst_pt, &t, &dist_sq))
      continue;
    if (dist_stats.add(dist_sq) & dist_stats.Smallest)
    {
      if (_clsst_pt != nullptr)
        *_clsst_pt = clsst_pt;
      if (_t != nullptr)
        *_t = t;
    }
  }
  if (dist_stats.count() == 0)
    return false;
  if (_dist_sq != nullptr)
    *_dist_sq = dist_stats.min();
  return true;
}

}//namespace Geo