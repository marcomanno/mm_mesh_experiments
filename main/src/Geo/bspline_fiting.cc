
#include <bspline_fiting.hh>
#include <Geo/iterate.hh>

#pragma warning( push )
#pragma warning( disable : 4714 )
#include <Eigen/Dense>

#include <fstream>
#include <vector>

namespace Geo {

template<size_t dimT>
struct BsplineFitting : public IBsplineFitting<dimT>
{
  bool init(const size_t _deg,
    const std::vector<double>& _knots, const IFunction& _f)
  {
    if (_knots.empty())
      return false;
    auto extn = (_knots.back() - _knots.front()) / 2;
    knots_.push_back(_knots.front() - extn);
    knots_.insert(knots_.end(), _knots.begin(), _knots.end());
    knots_.push_back(_knots.back() + extn);
    deg_ = _deg;
    f_ = &_f;
    X_.clear();
    A_.clear();
    B_.clear();
    return true;
  }

  void set_parameter_correction_iterations(size_t _itr_nmbr) { itr_nmbr_ = _itr_nmbr; }
  void set_favour_boundaries(const bool _fvr_bndr) { fvr_bndr_ = _fvr_bndr; }
  void set_samples_per_interval(const size_t _smpl_per_intrvl) 
  {
    smpl_per_intrvl_ = _smpl_per_intrvl;
  }

  void compute();
  const std::vector<VectorD<dimT>>& X() const { return X_; }
  VectorD<dimT> eval(const double _t);
private:
  void find_equations();
  double N(size_t _i, const size_t _k, const double _t);
  void add_equation(const double _t, const double _wi);

  std::vector<std::vector<double>> A_;
  std::vector<double> knots_;
  std::vector<VectorD<dimT>> B_;
  std::vector<VectorD<dimT>> X_;
  size_t deg_ = 0;
  const IFunction* f_ = nullptr;
  size_t itr_nmbr_ = 0;
  bool fvr_bndr_ = true;
  size_t smpl_per_intrvl_ = 4;
};

template<size_t dimT>
double BsplineFitting<dimT>::N(size_t _i, const size_t _p, const double _t)
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
void BsplineFitting<dimT>::add_equation(const double _t, const double _wi)
{
  A_.emplace_back();
  const auto wi_sqr = sqrt(_wi);
  for (int j = 0; j < knots_.size() - deg_ - 1; ++j)
    A_.back().push_back(N(j, deg_, _t) * wi_sqr);

  VectorD<dimT> pt_crv;
  if (X_.empty() || (_t == knots_[1]) || (_t == knots_[knots_.size() - 2]))
    pt_crv = f_->evaluate(_t);
  else
    pt_crv = f_->closest_point(eval(_t), _t);
  B_.emplace_back(pt_crv * wi_sqr);
};

template<size_t dimT>
void BsplineFitting<dimT>::find_equations()
{
  A_.clear();
  B_.clear();
  if (fvr_bndr_)
  {
    double w_prev = 0;
    const auto last_idx = knots_.size() - 2;
    for (size_t i = 2; i <= last_idx; ++i)
    {
      auto dw = knots_[i] - knots_[i - 1];
      if (dw <= 0)
        continue;
      const double step = 1. / smpl_per_intrvl_;
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
  }
  else
  {
    const auto last_idx = knots_.size() - 2;
    for (size_t i = 2; i <= last_idx; ++i)
    {
      auto dw = knots_[i] - knots_[i - 1];
      if (dw <= 0)
        continue;
      const double step = 1. / smpl_per_intrvl_;
      const double wi = dw * step;
      for (double x = step / 2; x < 1; x += step)
        add_equation(knots_[i - 1] * (1 - x) + knots_[i] * x, wi);
    }
  }
}

template<size_t dimT>
void BsplineFitting<dimT>::compute()
{
  for (size_t iter = 0; iter <= itr_nmbr_; ++iter)
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
VectorD<dimT> BsplineFitting<dimT>::eval(const double _t)
{
  VectorD<dimT> res = { 0 };
  for (int i = 0; i < X_.size(); ++i)
    res += N(i, deg_, _t) * X_[i];
  return res;
}

template<size_t dimT>
std::shared_ptr<IBsplineFitting<dimT>> IBsplineFitting<dimT>::make()
{
  return std::make_shared<BsplineFitting<dimT>>();
}

template struct IBsplineFitting<2>;
template struct IBsplineFitting<3>;

}//namespace Geo
