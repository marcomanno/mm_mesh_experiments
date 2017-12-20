#include "geo_function.hh"
#include "evalnurbs.hh"

namespace Geo
{

template <> double convert<double, std::array<double, 1>>(const std::array<double, 1>& pt)
{
  return pt[0];
}


template <size_t DimValT>
struct NubCurve : public Curve<DimValT>
{
  NubCurve(std::vector<Point>& _cpt, std::vector<double>& _knots):
    ctr_pts_(std::move(_cpt)), knots_(std::move(_knots))
  {
  }
  void evaluate(const Parameter& _par, Point& _val,
                std::vector<Derivative>* ders_ = nullptr,
                bool _right = false) override
  {
    using Evaluator = Nub<Point, double, Point>;
    Evaluator eval;
    if (!eval.init(ctr_pts_, knots_))
      throw "Bad nurvs data";
    size_t der_nmbr = 0;
    if (ders_ != nullptr && !ders_->empty())
      der_nmbr = ders_->back().der_order_[0];
    std::vector<Point> results(der_nmbr + 1);
    eval.eval(_par[0], results.begin(), results.end(), nullptr, _right);
    _val = results[0];
    if (ders_!= nullptr)
      for (auto& der : *ders_)
        der.val_ = results[der.der_order_[0]];
  }
private:
  std::vector<Point>   ctr_pts_;
  std::vector<double>  knots_;
};

template <size_t DimValT> std::shared_ptr<Curve<DimValT>> make_nurbs_curve(
  std::vector<VectorD<DimValT>>& _cpt, std::vector<double>& _knots)
{
  return std::make_shared<NubCurve<DimValT>>(_cpt, _knots);
}

template std::shared_ptr<Curve<2>> make_nurbs_curve<2>(std::vector<VectorD<2>>& _cpt, std::vector<double>& _knots);
template std::shared_ptr<Curve<3>> make_nurbs_curve<3>(std::vector<VectorD<3>>& _cpt, std::vector<double>& _knots);

} // namespace Geo