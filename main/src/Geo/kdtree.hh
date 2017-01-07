#pragma once

#include "iterate.hh"
#include "range.hh"
#include "vector.hh"

#include <assert.h>
#include <functional>
#include <list>
#include <vector>

namespace Geo {

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
  static const size_t DIM = 3;

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
  size_t leaf_start_ = 0;
  size_t leaf_lev_ = 0;

  Range<DIM> split(size_t _st, size_t _en, size_t _i)
  {
    auto en = _en;
    if (en > space_elements_.size())
      en = space_elements_.size();
    if (_en - _st <= LEAF_GROUP_SIZE || _st >= space_elements_.size())
    {
      Range<DIM> box;
      for (size_t i = _st; i < en; ++i) 
        box += space_elements_[i]->box();
      return box;
    }
    VectorD<DIM> ave = { 0 };
    for (auto i = _st; i < en; ++i)
      ave += space_elements_[i]->internal_point();
    ave /= double(en - _st);

    VectorD<DIM> sigma = { 0 };
    for (auto i = _st; i < en; ++i)
    {
      auto diff = space_elements_[i]->internal_point() - ave;
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
    { return _a->internal_point()[ind] < _b->internal_point()[ind]; });
    splits_[_i].split_val_ =
      ( space_elements_[mid_el]->internal_point()[ind] +
        space_elements_[mid_el - 1]->internal_point()[ind]) / 2;
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
    leaf_lev_ = static_cast<size_t>(std::ceil(log2(space_groups_nmbr)));
    size_t split_nmbr = static_cast<size_t>(std::exp2(leaf_lev_));
    leaf_start_ = split_nmbr - 1;
    splits_.resize(split_nmbr);
    box_ = split(0, split_nmbr * LEAF_GROUP_SIZE, 0);
    split_nmbr = splits_.size();
    for (; split_nmbr > 0; --split_nmbr)
      if (splits_[split_nmbr - 1].coord_index_ != INVALID)
        break;
    if (split_nmbr > 0)
      splits_.resize(split_nmbr);
    else
    {
      splits_.resize(1);
      splits_[0].box_[0] = box_;
      leaf_start_ = 1;
    }
  }

  const Range<DIM>& box() const { return box_; }

  const Range<DIM>& box(const std::array<size_t, 2>& _it) const
  {
    return splits_[_it[0]].box_[_it[1]];
  }

  size_t child_index(const std::array<size_t, 2>& _it) const
  {
    return _it[0] * GROUP_SIZE + _it[1] + 1;
  }

  bool child(std::array<size_t, 2>& _it) const
  {
    assert(_it[1] < GROUP_SIZE);
    auto child_ind = child_index(_it);
    if (child_ind >= splits_.size())
      return false;
    _it = std::array<size_t, 2>{child_ind, 0};
    return true;
  }

  bool next(std::array<size_t, 2>& _it) const
  {
    for (;;)
    {
      if (_it[1] == 0)
      {
        ++_it[1];
        return true;
      }
      if (_it[0] == 0)
        return false;
      _it[1] = (_it[0] & 1) ? 0 : 1;
      _it[0] = (_it[0] - 1) / 2;
    }
  }

  std::array<size_t, 2> next(
    std::array<size_t, 2> _it, bool& _leaf)
  {
    assert(_j < GROUP_SIZE);
    auto child_ind = child_index(_it);
    _leaf = child_ind >= splits_.size();
    return std::array<size_t, 2>{child_ind, 0};
  }

  size_t level_number() const { return leaf_lev_; }

  size_t depth() const { return splits_.size(); }

  bool leaf_range(const std::array<size_t, 2>& _it, std::array<size_t, 2>& _intrv) const
  {
    auto child_ind = child_index(_it);
    if (child_ind < leaf_start_)
      return false;
    _intrv[0] = (child_ind - leaf_start_) * LEAF_GROUP_SIZE;
    if (_intrv[0] >= space_elements_.size())
      return false;
    _intrv[1] = _intrv[0] + LEAF_GROUP_SIZE;
    if (_intrv[1] > space_elements_.size())
      _intrv[1] = space_elements_.size();
    return true;
  }

  const KdTreeElementT& operator[](size_t _i) const
  {
    return space_elements_[_i]; 
  }
};

template <class KdTreeElementT, class KdTreeElement1T = KdTreeElementT>
std::vector<std::array<size_t, 2>> find_kdtree_couples(
  KdTree<KdTreeElementT>& _kdt0,
  KdTree<KdTreeElement1T>& _kdt1)
{
  std::vector<std::array<size_t, 2>> coll_pairs;
  if ((_kdt0.box() * _kdt1.box()).empty())
    return coll_pairs;
  std::list<std::array<std::array<size_t, 2>, 2>> pairs;
  for (size_t j1 = 0; j1 < 2; ++j1)
    for (size_t j2 = 0; j2 < 2; ++j2)
    {
      std::array<std::array<size_t, 2>, 2> pair = { { { 0, j1 },{ 0, j2 } } };
      if (!(_kdt0.box(pair[0]) * _kdt1.box(pair[1])).empty())
        pairs.emplace_back(pair);
    }
  for (; !pairs.empty(); pairs.pop_front())
  {
    auto pair = pairs.front();
    bool has_child[] = { _kdt0.child(pair[0]), _kdt1.child(pair[1]) };
    if (!has_child[0] && !has_child[1])
    {
      std::array<std::array<size_t, 2>, 2> intrv;
      _kdt0.leaf_range(pair[0], intrv[0]);
      _kdt1.leaf_range(pair[1], intrv[1]);
      for (auto i = intrv[0][0]; i < intrv[0][1]; ++i)
        for (auto j = intrv[1][0]; j < intrv[1][1]; ++j)
          if (!(_kdt0[i]->box() * _kdt1[j]->box()).empty())
            coll_pairs.emplace_back(std::array<size_t, 2>{ i, j });
    }
    else
    {
      auto compare_box_and_add = [&_kdt0, &_kdt1, &pairs, &pair, &has_child]
      (const int _ch_ind = -1)
      {
        if (_ch_ind >= 0)
        {
          if (!has_child[_ch_ind])
            return false;
          pair[_ch_ind][1] = 1 - pair[_ch_ind][1];
        }
        if (!(_kdt0.box(pair[0]) * _kdt1.box(pair[1])).empty())
          pairs.emplace_back(pair);
        return true;
      };
      compare_box_and_add();
      if (compare_box_and_add(0))
      {
        if (compare_box_and_add(1))
          compare_box_and_add(0);
      }
      else
        compare_box_and_add(1);
    }
  }
  return coll_pairs;
}

}//namespace Geo