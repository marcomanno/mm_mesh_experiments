
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
  LSQ(const size_t _deg, const std::vector<double>& _knots, 
    Function<dimT>* _f) :  deg_(_deg), knots_(_knots), f_(_f)
  {
    find_equations();
  }
  void solve();
  const std::vector<Vector<dimT>>& X() const { return X_; }
  Vector<dimT> eval(const double _t);
private:
  void find_equations();
  double N(size_t _i, size_t _k, double _t);

  std::vector<std::vector<double>> A_;
  const std::vector<double>& knots_;
  std::vector<Vector<dimT>> B_;
  std::vector<Vector<dimT>> X_;
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
void LSQ<dimT>::find_equations()
{
  auto add_equation = [this](const double _t, const double _wi)
  {
    A_.emplace_back();
    const auto wi_sqr = sqrt(_wi);
    for (int j = 0; j < knots_.size() - deg_ - 1; ++j)
      A_.back().push_back(N(j, deg_, _t) * wi_sqr);
    B_.emplace_back(f_(_t) * wi_sqr);
  };
  double w_prev = 0;
  for (size_t i = 1; i < knots_.size(); ++i)
  {
    auto dw = knots_[i] - knots_[i - 1];
    if (dw <= 0)
      continue;
    const size_t SMPL_NMBR = 4;
    const double step = 1. / SMPL_NMBR;
    const double w_step = dw * step;
    auto wi = w_prev + w_step / 2;
    for (double x = 0; x < 1; x += step)
    {
      auto t = knots_[i - 1] * (1 - x) + knots_[i] * x;
      add_equation(t, wi);
      wi = w_step;
    }
    w_prev = w_step / 2;
  }
  add_equation(knots_.back(), w_prev);
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

template <size_t dimT> bool solve(
  const size_t _deg, Function<dimT>* _f,
  const std::vector<double>& _knots,
  std::vector<Vector<dimT>>& _opt_ctr_pts)
{
  std::vector<double> knots;
  knots.push_back(_knots.front());
  knots.insert(knots.end(), _knots.begin(), _knots.end());
  knots.push_back(_knots.back());
  LSQ<dimT> lsq(_deg, knots, _f);
  lsq.solve();
  _opt_ctr_pts = lsq.X();
  return true;
}

#define INSTANTIATE_SOLVE(N)             \
template bool solve<N>(                  \
  const size_t _deg, Function<N>* _f,    \
  const std::vector<double>& _knots,     \
  std::vector<Vector<N>>& _opt_ctr_pts);

INSTANTIATE_SOLVE(2)

INSTANTIATE_SOLVE(3)

}//namespace BsplineFItting
}//namespace Geo
