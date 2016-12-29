#pragma once

#include "iterate.hh"
#include "vector.hh"

#include <assert.h>
#include <functional>
#include <vector>

namespace Geo {

template <size_t DimT>
struct Range
{
  static Vector<DimT> point(double _v)
  {
    Vector<DimT> pt;
    for (auto& c : pt) c = _v;
    return pt;
  }
  // Empty range by default
  Vector<DimT> extr_[2] = {
    point(std::numeric_limits<double>::max()),
    point(std::numeric_limits<double>::lowest()) };

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

  // Union
  Range<DimT>& operator+=(const Range<DimT>& _oth)
  {
    return merge<std::greater<double>>(_oth);
  }
  const Range<DimT>& operator+(const Range<DimT>& _oth) const
  {
    return (Range<DimT>(*this) += _oth);
  }

  // Intersection
  Range<DimT>& operator*=(const Range<DimT>& _oth)
  {
    return merge<std::less<double>>(_oth);
  }
  const Range<DimT>& operator*(const Range<DimT>& _oth) const
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
      if (extr_[0][_i] > extr_[1][_i])
        return true;
    return false;
  }

  double diag_sq() const { return (extr_[0] - extr_[1]).len_sq(); }
  double diag() const { return std::sqrt(diag_sq()); }
};

// KdTreeElementT must have methods:
// const Vector3& point()
// const Range<3>& box()

struct KdTreeBase
{
  static const size_t GROUP_SIZE = 2;
  static const size_t LEAF_GROUP_SIZE = 4;
  static const size_t INVALID = 
    std::numeric_limits<size_t>::max();
};


template <class KdTreeElementT>
class KdTree : public KdTreeBase
{
  static const size_t DIM = KdTreeElementT::DIM;

  std::vector<KdTreeElementT> space_elements_;
  struct Split
  {
    size_t coord_index_ = INVALID;
    double split_val_;
    Range<DIM> box_[GROUP_SIZE]; // left and right ranges.
  };
  // N               Nwe elem      (GN ^ n - 1) / (GN - 1)
  // 0) 0              1                    0
  // 1) 1 .. 4 =>      4                    1
  // 2) 5 .. 20 =>    16                    5
  // 3) 21 .. 84 =>   64                   21
  // 4) 85 .. 343 => 256                   85
  // Given an index i, find 
  std::vector<Split> splits_;
  Range<DIM> box_;

  Range<3> split(size_t _st, size_t _en, size_t _i)
  {
    auto en = _en;
    if (en > space_elements_.size())
      en = space_elements_.size();
    if (en - _st <= LEAF_GROUP_SIZE)
    {
      Range<3> box;
      for (size_t i = _st; i < en; ++i) 
        box += space_elements_[i].box();
      return box;
    }
    Vector3 ave = { 0 };
    for (auto i = _st; i < en; ++i)
      ave += space_elements_[i].point();
    ave /= double(en - _st);

    Vector3 sigma = { 0 };
    for (auto i = _st; i < en; ++i)
    {
      auto diff = space_elements_[i].point() - ave;
      iterate_forw<diff.size()>::eval([&sigma, &diff](size_t _j)
      { sigma[_j] += std::pow(diff[_j], 2); });
    }
    auto ind = splits_[_i].coord_index_ =
      std::max_element(sigma.begin(), sigma.end()) - sigma.begin();
    auto dat = space_elements_.begin();
    auto mid_el = (_en + _st) / 2;
    if (mid_el >= en)
    {
      return splits_[_i].box_[0] = split(_st, mid_el, 2 * _i + 1);
    }
    std::nth_element(dat + _st, dat + mid_el, dat + en,
      [ind](const KdTreeElementT& _a, const KdTreeElementT& _b)
    { return _a.point()[ind] < _b.point()[ind]; });
    splits_[_i].split_val_ =
      ( space_elements_[mid_el].point()[ind] +
        space_elements_[mid_el - 1].point()[ind]) / 2;
    splits_[_i].box_[0] = split(_st, mid_el, 2 * _i + 1);
    splits_[_i].box_[1] = split(mid_el, _en, 2 * _i + 2);
    return splits_[_i].box_[0] + splits_[_i].box_[1];
  }

public:
  template <typename IteratorT>
  void insert(IteratorT _beg, IteratorT _end)
  {
    splits_.clear();
    while(_beg != _end)
      space_elements_.push_back(*_beg++);
  }
  void compute()
  {
    size_t space_groups_nmbr =
      (space_elements_.size() + LEAF_GROUP_SIZE - 1) / LEAF_GROUP_SIZE;
    size_t leaf_lev = static_cast<size_t>(std::ceil(log2(space_groups_nmbr)));
    size_t split_nmbr = static_cast<size_t>(std::exp2(leaf_lev + 1));
    splits_.resize(split_nmbr * 2);
    box_ = split(0, split_nmbr * LEAF_GROUP_SIZE, 0);
    auto nsplit = splits_.size();
    while (nsplit-- > 0)
    {
      if (splits_[nsplit].coord_index_ != INVALID)
      {
        splits_.resize(++nsplit);
        break;
      }
    }
  }
  Range<3> box(size_t _i, size_t _j)
  {
    return splits_[_i].box_[_j];
  }
  size_t child(size_t _i, size_t _j)
  {
    assert(_j < GROUP_SIZE);
    return _i * GROUP_SIZE + 1 + _j;
  }

  size_t depth() const { return splits_.size(); }

  void leaf_range(const size_t _i, size_t& _start, size_t& _end)
  {
    auto leaf_start = static_cast<size_t>(
      std::exp2(
        std::floor(
          std::log2(_i + 1))) - 1);
    _start = (_i - leaf_start) * LEAF_GROUP_SIZE;
    assert(_start < space_elements_.size());
    _end = _start + LEAF_GROUP_SIZE;
    if (_end > space_elements_.size())
      _end = space_elements_.size();
  }

  const KdTreeElementT& operator[](size_t _i) const
  {
    return space_elements_[_i]; 
  }
};

}//namespace Geo