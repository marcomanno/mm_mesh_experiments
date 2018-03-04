#include "plane_fitting.hh"
#include "iterate.hh"
#include <Utils/error_handling.hh>
#include "Eigen/SVD"
#include <vector>


namespace Geo {

namespace {

struct PlaneFit : public IPlaneFit
{
  virtual void init(size_t _size) override;
  virtual void add_point(const VectorD3& _pt) override;
  virtual bool compute(
    VectorD3& _center, 
    VectorD3& _normal,
    const bool _orient = false) override;

  Eigen::Matrix<double, 3, Eigen::Dynamic> matr_;
  int ind_ = 0;
};

void PlaneFit::init(size_t _size)
{
  matr_.resize(3, _size);
  ind_ = 0;
}

void PlaneFit::add_point(const VectorD3& _pt)
{
  THROW_IF(ind_ > matr_.cols(), "Adding too many points");
  iterate_forw<3>::eval([this, &_pt](size_t _i) {matr_(_i, ind_) = _pt[_i]; });
  ++ind_;
}

// Find the best plane for n points using the singular value decomposition.
// This is where I took the idea:
// http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points?answertab=votes#tab-top
bool PlaneFit::compute(
  VectorD3& _center, VectorD3& _normal, const bool _orient)
{
  if (ind_ == 0)
    return false;
  if (ind_ < matr_.cols())
    matr_.conservativeResize(Eigen::NoChange, ind_);

  Eigen::Vector3d mid_pt(0, 0, 0);

  for (int i = 0; i < matr_.cols(); ++i)
    mid_pt += matr_.col(i);

  mid_pt /= double(matr_.cols());

  iterate_forw<3>::eval([&_center, &mid_pt](int _i) { _center[_i] = mid_pt(_i, 0); });

  for (int i = 0; i < matr_.cols(); ++i)
    matr_.col(i) -= mid_pt;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matr_, Eigen::ComputeThinU);
  auto umatr = svd.matrixU();
  const auto cols = umatr.cols();
  if (cols == 0)
    return false;
  iterate_forw<3>::eval([&_normal, &umatr, cols](int _i) 
  { _normal[_i] = umatr(_i, cols - 1); });
  if (_orient) // Normal in ccw.
  {
    VectorD3 du, dv;
    normal_plane_default_directions(_normal, du, dv);
    auto proj_to_plane = [this, &du, &dv](size_t i)
    {
      VectorD3 p = { matr_(0, i), matr_(1, i), matr_(2, i) };
      return VectorD2{p * du, p * dv};
    };
    auto pt_ptrv = proj_to_plane(matr_.cols() - 1);
    double area = 0;
    for (std::ptrdiff_t i = 0; i < matr_.cols(); ++i)
    {
      auto pt = proj_to_plane(i);
      area += (pt[1] + pt_ptrv[1]) * (pt_ptrv[0] - pt[0]) / 2;
      pt_ptrv = pt;
    }
    if (area < 0)
      _normal *= -1.;
  }
  return true;
}

}//namespace

std::shared_ptr<IPlaneFit> IPlaneFit::make()
{
  return std::make_shared<PlaneFit>();
}

}//namespace Geo
