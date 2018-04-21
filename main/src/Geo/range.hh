#pragma once

#include "vector.hh"
#include "iterate.hh"

namespace Geo {

template <class TypeT>
class Interval
{
  Vector<TypeT, 2> extr_;
public:
  Interval(const TypeT& _a = std::numeric_limits<TypeT>::max(),
           const TypeT& _b = std::numeric_limits<TypeT>::lowest())
    : extr_({_a, _b}) {}

  Interval(const Interval<TypeT>& _int) : extr_(_int.extr_) {}

  bool empty() const { return extr_[0] >= extr_[1]; }

  void set_empty()
  {
    extr_[0] = std::numeric_limits<TypeT>::max();
    extr_[1] = std::numeric_limits<TypeT>::lowest();
  }

  static TypeT max() { return std::numeric_limits<TypeT>::max(); }
  static TypeT min() { return std::numeric_limits<TypeT>::lowest(); }

  void set_whole()
  {
    set_empty();
    std::swap(extr_[1], extr_[0]);
  }

  void set(bool _max, const TypeT& _val)
  {
    extr_[_max] = _val;
  }

  const TypeT& operator[](size_t _i) const
  {
    return extr_[_i];
  }

  bool contain_close(const TypeT& _val) const
  {
    return _val >= extr_[0] && _val <= extr_[1];
  }

  bool contain_close(const Interval<TypeT>& _oth) const
  {
    return extr_[0] <= _oth.extr_[0] && extr_[1] >= _oth.extr_[1];
  }
    
  bool contain_open(const TypeT& _val) const
  {
    return _val > extr_[0] && _val < extr_[1];
  }

  bool contain_open(const Interval<TypeT>& _oth) const
  {
    return extr_[0] < _oth.extr_[0] && extr_[1] > _oth.extr_[1];
  }

  void add(const TypeT& _val)
  {
    if (extr_[0] > _val) extr_[0] = _val;
    if (extr_[1] < _val) extr_[1] = _val;
  }
  void add(const Interval<TypeT>& _int)
  {
    add(_int.extr_[0]);
    add(_int.extr_[1]);
  }
  bool split(const TypeT& _val, Interval<TypeT>& _oth)
  {
    if (!contain_open(_val))
      return false;
    _oth = Interval<TypeT>(_val, extr_[1]);
    extr_[1] = _val;
    return true;
  }

  // Intersection
  Interval<TypeT>& operator*=(const Interval<TypeT>& _oth)
  {
    if (extr_[0] < _oth.extr_[0]) extr_[0] = _oth.extr_[0];
    if (extr_[1] > _oth.extr_[1]) extr_[1] = _oth.extr_[1];
    return *this;
  }
  Interval<TypeT> operator*(const Interval<TypeT>& _oth) const
  {
    return (Interval<TypeT>(*this) *= _oth);
  }
  Interval<TypeT>& operator+=(const Interval<TypeT>& _oth)
  {
    if (extr_[0] > _oth.extr_[0]) extr_[0] = _oth.extr_[0];
    if (extr_[1] < _oth.extr_[1]) extr_[1] = _oth.extr_[1];
    return *this;
  }
  Interval<TypeT> operator+(const Interval<TypeT>& _oth) const
  {
    return (Interval<TypeT>(*this) += _oth);
  }

  TypeT length() const { return extr_[1] - extr_[0]; }

  TypeT interpolate(TypeT _par) const { return _par * extr_[1] + (1 - _par) *  extr_[0]; }

  bool subtract(const Interval<TypeT>& _oth, Interval<TypeT>& _part2)
  {
    _part2.set_empty();
    if (_oth.contain_close(*this))
    {
      set_empty();
      return true;
    }
    if (contain_open(_oth))
    {
      split(_oth.extr_[1], _part2);
      Interval<TypeT> part2;
      split(_oth.extr_[0], part2);
      return true;
    }
    if (contain_open(_oth.extr_[1]))
      extr_[0] = _oth.extr_[1];
    else if (contain_open(_oth.extr_[0]))
      extr_[1] = _oth.extr_[0];
    return true;
  }

};

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

  void clear()
  {
    *this = Range();
  }

  bool operator<(const Range<DimT>& _oth) const
  {
    auto i = extr_[0] == extr_[0] ? 1: 0;
    return extr_[i] < extr_[i]
  }

  bool operator==(const Range<DimT>& _oth) const
  {
    return extr_[0] == _oth.extr_[0] && extr_[1] == _oth.extr_[1];
  }
  bool operator!=(const Range<DimT>& _oth) const
  {
    return !(_oth == *this);
  }

  double diag_sq() const { return (extr_[0] - extr_[1]).len_sq(); }
  double diag() const { return std::sqrt(diag_sq()); }

  void set(bool _max, const VectorD<DimT>& _extr)
  {
    extr_[_max] = _extr;
  }

  const VectorD<DimT>& operator[](int _i) const
  {
    return extr_[_i];
  }

  bool contain(const VectorD<DimT>& _pt) const
  {
    for (int i = 0; i < _pt.size(); ++i)
      if (_pt[i] < extr_[0][i] || extr_[0][i] < _pt[i])
        return false;
    return true;
  }
};

template <size_t DimT, class Container>
Range<DimT> make_range(const Container& _cont)
{
  Range<DimT> range;
  for (auto it = _cont.begin(); it != _cont.end(); ++it)
    range += Geo::Vector<double, DimT>{*it};
  return range;
}

}// namespace