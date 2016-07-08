#pragma once

#include "index.hh"

#include <vector>

namespace Utils {

struct Mergiable
{
  Index equiv_idx_ = INVALID_INDEX;
  // Class Data must provide these two methods:
  //bool equivalent(const Data& _oth) const;
  //bool merge(MergiableT<Data>& _oth) = 0;
};

template <class MergiableT>
void merge(std::vector<MergiableT>& _vec)
{
  for (size_t i = 0; i < _vec.size(); ++i)
  {
    for (size_t j = i; ++j < _vec.size(); )
    {
      if (!_vec[i].equivalent(_vec[j]))
        continue;
      if (_vec[j].equiv_idx_ == INVALID_INDEX)
        _vec[j].equiv_idx_ = i;
      else if (_vec[i].equiv_idx_ == INVALID_INDEX)
        _vec[i].equiv_idx_ = j;
      else
      {
        Index k = j;
        for (Index k_next; _vec[k].equiv_idx_ != INVALID_INDEX; k = k_next)
        {
          k_next = _vec[k].equiv_idx_;
          _vec[k].equiv_idx_ = _vec[i].equiv_idx_;
        }
      }
    }
  }
  for (size_t i = 0; i < _vec.size(); ++i)
  {
    if (_vec[i].equiv_idx_ == INVALID_INDEX)
      continue;
    auto k = i;
    while(_vec[k].equiv_idx_ != INVALID_INDEX)
      k = _vec[k].equiv_idx_;
    _vec[i].equiv_idx_ = k;
  }
  for (size_t i = 0; i < _vec.size(); ++i)
  {
    auto base_idx = _vec[i].equiv_idx_;
    if (base_idx == INVALID_INDEX)
      continue;
    _vec[base_idx].merge(_vec[i]);
  }
}

}//Utils