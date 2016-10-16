
#include <bspline_fiting.hh>

#pragma warning( push )
#pragma warning( disable : 4714 )
#include <Eigen/Dense>

#include <fstream>
#include <vector>


namespace Geo {
namespace BsplineFItting {
namespace {

template<size_t dimT>
struct LSQ
{
  LSQ(const size_t _refn_lev,
    const size_t _deg,
    const std::vector<double>& _knots,
    Function<dimT>* _f) :
    refn_lev_(_refn_lev), deg_(_deg), knots_(_knots), f_(_f)
  {
    find_equations(0);
  }
  void solve();
  const std::vector<Vector<dimT>>& X() const { return X_; }
  Vector<dimT> eval(const double _t);
private:
  void find_equations(int _refn, const double _next_wi = 0);
  double N(size_t _i, size_t _k, double _t);

  std::vector<std::vector<double>> A_;
  const std::vector<double>& knots_;
  std::vector<Vector<dimT>> B_;
  std::vector<Vector<dimT>> X_;
  const size_t refn_lev_;
  const size_t deg_;
  Function<dimT>* f_;
};

template<size_t dimT>
double LSQ<dimT>::N(size_t _i, size_t _p, double _t)
{
  if (_p == 0)
    return _t >= knots_[_i] && _t < knots_[_i + 1] ? 1. : 0.;
  double res = 0;

  auto b = N(_i, _p - 1, _t);
  if (b != 0)
    res += b * (_t - knots_[_i]) / (knots_[_i + _p] - knots_[_i]);

  b = N(_i + 1, _p - 1, _t);
  if (b != 0)
  {
    auto end_kn = knots_[_i + _p + 1];
    res += b * (end_kn - _t) / (end_kn - knots_[_i + 1]);
  }
  return res;
}

template<size_t dimT>
void LSQ<dimT>::find_equations(int _refn, const double _next_wi)
{
  const double step = 0.25;
  const auto refn_fctor = std::pow(2, -_refn);
  const double  w = step * refn_fctor;
  for (double x = 0; x < 1; x += step)
  {
    double  wi = w;
    if (x == 0)
      wi /= 2;
    if (x < 0.5 && _refn < refn_lev_)
      find_equations(_refn + 1, wi);
    else
    {
      if (x == 1)
        wi = wi / 2 + _next_wi;
      const auto t = x * refn_fctor;
      A_.emplace_back();
      for (int i = 0; i < knots_.size() - deg_ - 1; ++i)
        A_.back().push_back(N(i, deg_, t) * wi);
      B_.emplace_back(f_(t) * wi);
    }
  }
}

template<size_t dimT>
void LSQ<dimT>::solve()
{
  const auto row_nmbr = A_.size();
  const auto col_nmbr = A_.front().size();
  Eigen::MatrixXd A(row_nmbr, col_nmbr);
  for (int i = 0; i < row_nmbr; ++i)
  {
    for (int j = 0; j < col_nmbr; ++j)
      A(i, j) = A_[i][j];
  }
  const auto& jsvd =
    A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  for (int j = 0; j < B_[0].size(); ++j)
  {
    Eigen::VectorXd B(B_.size());
    for (int i = 0; i < row_nmbr; ++i)
      B(i) = B_[i][j];
    Eigen::VectorXd res = jsvd.solve(B);
    X_.resize(col_nmbr);
    for (int i = 0; i < col_nmbr; ++i)
      X_[i][j] = res(i);
  }
}

template<size_t dimT>
Vector<dimT> LSQ<dimT>::eval(const double _t)
{
  Vector<dimT> res;
  for (int i = 0; i < X_.size(); ++i)
    res += N(i, deg_, _t) * X_[i];
  return res;
}

}//namespace

#if 0
void minimize()
{
  pts.resize(DEG + 1);
  LSQ lsq(0);
  lsq.solve();
  std::ofstream bspl("b0.txt");
  for (auto k : knots_)
    bspl << " " << k;
  bspl << std::endl;
  for (const auto& pt : lsq.X)
    for (const auto& coo : pt)
      bspl << " " << coo;
  bspl << std::endl;
}
#endif

template <size_t dimT> bool solve(
  const size_t _deg, const size_t _ref_lev,
  Function<dimT>* _f,
  std::vector<double>& _knots,
  std::vector<Vector<dimT>>& _opt_ctr_pts)
{
  std::vector<double> knots(_deg + 1, 0.);
  for (auto ref = _ref_lev; ref > 0; --ref)
    knots.push_back(std::pow(2., -double(ref)));
  knots.insert(knots.end(), _deg + 1, 1.);
  LSQ<dimT> lsq(_ref_lev, _deg, knots, _f);
  lsq.solve();
  _opt_ctr_pts = lsq.X();
  _knots.assign(std::next(knots.begin()), std::prev(knots.end()));
  return true;
}

#define INSTANTIATE_SOLVE(N)                \
template bool solve<N>(                     \
  const size_t _deg, const size_t _ref_lev, \
  Function<N>* _f,                          \
  std::vector<double>& _knots,              \
  std::vector<Vector<N>>& _opt_ctr_pts);

INSTANTIATE_SOLVE(2)

INSTANTIATE_SOLVE(3)

}//namespace BsplineFItting
}//namespace Geo
