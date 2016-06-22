#pragma once

#include <limits>

namespace Geo
{
template <typename ValueT> class StatisticsT
{
  ValueT max_ = std::numeric_limits<ValueT>::lowest();
  ValueT min_ = std::numeric_limits<ValueT>::max();
  ValueT sum_ = 0;
  size_t count_ = 0;
  size_t max_idx_ = 0, min_idx_ = 0;
public:
  void add(const ValueT& _val)
  {
    if (max_ < _val)
    {
      max_ = _val;
      max_idx_ = count_;
    }
    if (_val < min_)
    {
      min_ = _val;
      min_idx_ = count_;
    }
    sum_ += _val;
    ++count_;
  }
  const ValueT& max() const { return max_; }
  const ValueT& min() const { return min_; }
  const size_t& max_idx() const { return max_idx_; }
  const size_t& min_idx() const { return min_idx_; }
  const ValueT& sum() const { return sum_; }
  const ValueT& average() const { return sum_ / count_; }
};

}//namespace Geo
