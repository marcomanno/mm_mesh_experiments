#pragma once

#include "vector.hh"
#include "iterate.hh"

namespace Geo {

template <size_t DimT>
struct Range
{
  // Empty range by default
  VectorD<DimT> extr_[2] = {
    uniform_vector<DimT>(std::numeric_limits<double>::max()),
    uniform_vector<DimT>(std::numeric_limits<double>::lowest()) };

  template <typename CompareT>
  Range<DimT>& merge(const Range<DimT>& _oth)
  {
    iterate_forw<DimT>::eval([this, &_oth](const int _i)
    {
      if (CompareT()(extr_[0][_i], _oth.extr_[0][_i]))
        extr_[0][_i] = _oth.extr_[0][_i];
      if (CompareT()(_oth.extr_[1][_i], extr_[1][_i]))
        extr_[1][_i] = _oth.extr_[1][_i];
    });
    return *this;
  }
public:

  // Add point
  Range<DimT>& operator+=(const VectorD<DimT>& _pt)
  {
    iterate_forw<DimT>::eval([this, &_pt](const int _i)
    {
      if (extr_[0][_i] > _pt[_i])
        extr_[0][_i] = _pt[_i];
      if (extr_[1][_i] < _pt[_i])
        extr_[1][_i] = _pt[_i];
    });
    return *this;
  }
  Range<DimT> operator+(const VectorD<DimT>& _pt) const
  {
    return (Range<DimT>(*this) += _pt);
  }

  // Union
  Range<DimT>& operator+=(const Range<DimT>& _oth)
  {
    return merge<std::greater<double>>(_oth);
  }
  Range<DimT> operator+(const Range<DimT>& _oth) const
  {
    return (Range<DimT>(*this) += _oth);
  }

  // Intersection
  Range<DimT>& operator*=(const Range<DimT>& _oth)
  {
    return merge<std::less<double>>(_oth);
  }
  Range<DimT> operator*(const Range<DimT>& _oth) const
  {
    return (Range<DimT>(*this) *= _oth);
  }

  void fatten(const double _tol)
  {
    iterate_forw<DimT>::eval([this, _tol](const size_t _i)
    {
      extr_[0][_i] -= _tol;
      extr_[1][_i] += _tol;
    });
  }

  bool empty() const
  {
    for (auto i = 0; i < DimT; ++i)
      if (extr_[0][i] > extr_[1][i])
        return true;
    return false;
  }

  double diag_sq() const { return (extr_[0] - extr_[1]).len_sq(); }
  double diag() const { return std::sqrt(diag_sq()); }
};

}// namespace