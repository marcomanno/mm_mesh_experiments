
#include <bspline_fiting.hh>
#include <Geo/iterate.hh>

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
    Function<dimT>* _f) :  deg_(_deg), knots_(_knots), f_(_f) { }
  void solve();
  const std::vector<Vector<dimT>>& X() const { return X_; }
  Vector<dimT> eval(const double _t);
private:
  void find_equations();
  double N(size_t _i, const size_t _k, const double _t);
  void add_equation(const double _t, const double _wi);
  double parameter_correction(const double _t);

  std::vector<std::vector<double>> A_;
  const std::vector<double>& knots_;
  std::vector<Vector<dimT>> B_;
  std::vector<Vector<dimT>> X_;
  const size_t deg_;
  Function<dimT>* f_;
};

template<size_t dimT>
double LSQ<dimT>::N(size_t _i, const size_t _p, const double _t)
{
  if (_p == 0)
  {
    if (_t < knots_[_i] || _t >= knots_[_i + 1])
      return 0;
    return 1;
  }
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
double LSQ<dimT>::parameter_correction(const double _t)
{
  if (X_.empty() || _t <= 0 || _t >= 1)
    return _t;
  auto t = _t;
  Geo::Vector<dimT> pt_bspl = eval(_t);
  for (size_t i = 0; i < 5; ++i)
  {
    Geo::Vector<dimT> der;
    auto cv_pt = f_(t, &der);
    auto dp = pt_bspl - cv_pt;
    t += (dp * der) / Geo::length_square(der);
    if (t < 0)
      t = 0;
    if (t > 1)
      t = 1;
  }
  return t;
}

template<size_t dimT>
void LSQ<dimT>::add_equation(const double _t, const double _wi)
{
  A_.emplace_back();
  const auto wi_sqr = sqrt(_wi);
  for (int j = 0; j < knots_.size() - deg_ - 1; ++j)
    A_.back().push_back(N(j, deg_, _t) * wi_sqr);
  B_.emplace_back(f_(parameter_correction(_t), nullptr) * wi_sqr);
};

template<size_t dimT>
void LSQ<dimT>::find_equations()
{
  A_.clear();
  B_.clear();
  const size_t SMPL_NMBR = 4;
  for (size_t iter = 0; iter < 5; ++iter)
  {
#define TRAP_BOUNDARY
#ifdef TRAP_BOUNDARY
    double w_prev = 0;
    const auto last_idx = knots_.size() - 2;
    for (size_t i = 2; i <= last_idx; ++i)
    {
      auto dw = knots_[i] - knots_[i - 1];
      if (dw <= 0)
        continue;
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
    add_equation(knots_[last_idx], w_prev);
#else
    const auto last_idx = knots_.size() - 2;
    for (size_t i = 2; i <= last_idx; ++i)
    {
      auto dw = knots_[i] - knots_[i - 1];
      if (dw <= 0)
        continue;
      const double step = 1. / SMPL_NMBR;
      const double wi = dw * step;
      for (double x = step / 2; x < 1; x += step)
        add_equation(knots_[i - 1] * (1 - x) + knots_[i] * x, wi);
    }
#endif
  }
}

template<size_t dimT>
void LSQ<dimT>::solve()
{
  for (size_t iter = 0; iter < 5; ++iter)
    {
      find_equations();
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
}

template<size_t dimT>
Vector<dimT> LSQ<dimT>::eval(const double _t)
{
  Vector<dimT> res = { 0 };
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
  if (_knots.size() == 0)
    return false;
  double dt = _knots.back() - _knots.front();
  std::vector<double> knots;
  knots.push_back(_knots.front() - dt);
  knots.insert(knots.end(), _knots.begin(), _knots.end());
  knots.push_back(_knots.back() + dt);
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
