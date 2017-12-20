#pragma once
#include "range.hh"

#include <memory>
#include <vector>

namespace Geo
{
template <size_t DimPartT, size_t DimValT> struct MathFunc
{
  using Parameter = Geo::Vector<double, DimPartT>;
  using Point = Geo::Vector<double, DimValT>;
  struct Derivative
  {
    std::array<size_t, DimPartT> der_order_ = { 1 };
    Point val_;
  };
  virtual void evaluate(const Parameter& _par, Point& _val,
                        std::vector<Derivative>* ders_ = nullptr, 
                        bool _right = false) = 0;
  virtual const Geo::Range<DimPartT>& range() final { return range_; }
  virtual const Geo::Range<DimValT>& box() final { return box_; }
protected:
  Geo::Range<DimPartT> range_;
  Geo::Range<DimValT> box_;
};

template <size_t DimValT> using Curve = Geo::MathFunc<1, DimValT>;
using Surface = MathFunc<2, 3>;

template <size_t DimPartT, size_t DimValT, class T> 
T* function_cast(MathFunc<DimPartT, DimValT>* _mf)
{
  return nullptr;
}

template <size_t DimValT>
std::shared_ptr<Curve<DimValT>> make_nurbs_curve(
  std::vector<VectorD<DimValT>>& _cpt, std::vector<double>& _knots);
}// namespace Geo
