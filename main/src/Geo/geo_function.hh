#pragma once
#include "range.hh"

#include <memory>
#include <vector>

namespace Geo
{

template <size_t DimPartT, size_t DimValT> struct Derivative
{
  std::array<size_t, DimPartT> der_order_ = { 0 };
  Geo::Vector<double, DimValT> val_;
};

template <size_t DimPartT, size_t DimValT> struct MathFunc
{
  virtual void evaluate(const Geo::Vector<double, DimPartT>& _par,
                        Geo::Vector<double, DimValT>& _val,
                        std::vector<Derivative<1, DimValT>>* ders_ = nullptr,
                        bool _right = false) = 0;
  virtual void curvature(const Geo::Vector<double, DimPartT>& _par, 
                         Geo::Vector<double, DimValT>& _val) = 0;
  virtual void torsion(const Geo::Vector<double, DimPartT>& _par,
                       Geo::Vector<double, DimValT>& _val) = 0;
  virtual double max_derivative(int _order) = 0;
  virtual Geo::Range<DimPartT> range() = 0;
  virtual Geo::Range<DimValT> box() = 0;
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
