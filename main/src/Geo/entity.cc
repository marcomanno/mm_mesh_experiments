
#include <entity.hh>
#include <vector.hh>
#include <pow.hh>
#include <linear_system.hh>
#include <Utils/statistics.hh>
#include "PolygonTriangularization/poly_triang.hh"
#include "Utils/error_handling.hh"

#define JACOBI
#ifdef JACOBI
#pragma warning( push )
#pragma warning( disable : 4714 )
#include <Eigen/Dense>
#endif

#include <vector>

namespace {
bool check_par(double _t) { return _t >= 0 && _t <= 1; }
bool check_par(double _u, double _v)
{
  return _u >= 0 && _v >= 0 && (1 - _u - _v) >= 0;
}
}//namespace

namespace Gen
{
template <class TypeT, size_t DimT, bool Inf1T, bool Inf2T>
bool closest_point(const Segment<TypeT, DimT>& _seg_a, const Segment<TypeT, DimT>& _seg_b,
                   Geo::Vector<TypeT, DimT>* _clsst_pt, double _t[2], double * _dist_sq)
{
  Geo::Vector<TypeT, DimT> a[2] = {
    _seg_a[1] - _seg_a[0],
    _seg_b[0] - _seg_b[1] };
  Eigen::MatrixXd A(a[0].size(), std::size(a));
  Eigen::VectorXd B(a[0].size());
  for (int i = 0; i < a[0].size(); ++i)
  {
    B(i) = _seg_b[0][i] - _seg_a[0][i];
    for (int j = 0; j < std::size(a); ++j)
      A(i, j) = a[j][i];
  }
  Eigen::VectorXd  res = (A.transpose() * A).ldlt().solve(A.transpose() * B);
  if (constexpr(!Inf1T))
  {
    if (!check_par(res(0)))
      return false;
  }
  if (constexpr(!Inf2T))
  {
    if (!check_par(res(1)))
      return false;
  }
  auto pt_a = evaluate(_seg_a, res(0));
  auto pt_b = evaluate(_seg_b, res(1));
  if (_clsst_pt != nullptr)
    *_clsst_pt = (pt_a + pt_b) / 2.;
  if (_dist_sq != nullptr)
    *_dist_sq = Geo::length_square(pt_a - pt_b);
  if (_t != nullptr)
  {
    _t[0] = res(0);
    _t[1] = res(1);
  }
  return true;
}

#define INST_CLOSEST_POINT_SEG_SEG(TYPE, NUM, INF)                        \
template bool closest_point<TYPE, NUM, INF, INF>(                              \
  const Segment<TYPE, NUM>& _seg_a, const Segment<TYPE, NUM>& _seg_b,\
  Geo::Vector<TYPE, NUM>* _clsst_pt, double _t[2], double * _dist);

INST_CLOSEST_POINT_SEG_SEG(double, 2, false)
INST_CLOSEST_POINT_SEG_SEG(double, 2, true)

INST_CLOSEST_POINT_SEG_SEG(double, 3, false)
INST_CLOSEST_POINT_SEG_SEG(double, 3, true)

} // namespace Gen

namespace Geo {

Point evaluate(const Segment& _seg, double _t)
{
  return (1 - _t) * _seg[0] + _t * _seg[1];
}

Point evaluate(const Triangle& _tri, double _u, double _v)
{
  return (1 - _u - _v) * _tri[0] +_u * _tri[1] + _v * _tri[2];
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
  virtual size_t make_new_loop() override
  {
    ptss_.emplace_back();
    return ptss_.size() - 1;
  }
  virtual void add_point(const Point& _pt, size_t _loop_num) override
  {
    ptss_[_loop_num].push_back(_pt);
  }
  virtual void compute() override;

private:
  std::vector<Triangle> tris_;
  std::vector<std::vector<Point>> ptss_;
};

void PolygonalFace::compute()
{
  if (ptss_.empty())
    return;
  auto ptg = IPolygonTriangulation::make();
  for (auto& pts : ptss_)
  {
    THROW_IF(pts.size() < 3, "Loop withless than 3 points.");
    ptg->add(pts);
  }
  const auto& tris = ptg->triangles();
  const auto& poly = ptg->polygon();
  for (const auto& tri : tris)
  {
    tris_.push_back({
      poly[tri[0]],
      poly[tri[1]],
      poly[tri[2]] });
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

std::shared_ptr<IPolygonalFace> IPolygonalFace::make()
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
  const auto tri_nmbr = _face.triangle_number();
  for (size_t i = 0; i < tri_nmbr; ++i)
  {
    if (!_face.triangle(i, tri))
      continue;
    double dist_sq = 0;
    if (!closest_point(tri, _pt, &clsst_pt, &dist_sq))
      continue;
    if ((dist_stats.add(dist_sq) & dist_stats.Smallest) && _clsst_pt != nullptr)
      *_clsst_pt = clsst_pt;
  }
  if (dist_stats.count() == 0)
    return false;
  if (_dist_sq != nullptr)
    *_dist_sq = dist_stats.min();
  return true;
}

bool closest_point(const Triangle& _tri, const Segment& _seg,
  Point* _clsst_pt, double * _t, double * _dist_sq)
{
  const auto a = _seg[0] - _tri[0];
  const Point coeff[] =
  { _tri[1] - _tri[0], _tri[2] - _tri[0], _seg[0] - _seg[1] };

  Eigen::MatrixXd A(3, 3);
  Eigen::VectorXd B(3);
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
      A(j, i) = coeff[i][j];
    B(i) = a[i];
  }
#if 1
  Eigen::VectorXd  uvt = (A.transpose() * A).ldlt().solve(A.transpose() * B);
#else
  const auto& jsvd =
    A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd uvt = jsvd.solve(B);
#endif

  if (!check_par(uvt[0], uvt[1]) || !check_par(uvt[2]))
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