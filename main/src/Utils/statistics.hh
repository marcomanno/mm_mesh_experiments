#pragma once

#include <limits>

namespace Utils
{
template <typename ValueT> class StatisticsT
{
  ValueT max_ = std::numeric_limits<ValueT>::lowest();
  ValueT min_ = std::numeric_limits<ValueT>::max();
  ValueT sum_ = 0;
  size_t count_ = 0;
  static const size_t INVALID = std::numeric_limits<size_t>::max();
  size_t max_idx_ = 0, min_idx_ = 0;
public:
  enum classification {Neutral = 0, Biggest = 1, Smallest = 2};
  size_t add(const ValueT& _val, size_t _i = INVALID)
  {
    size_t ret = Neutral;
    if (max_ < _val)
    {
      max_ = _val;
      max_idx_ = _i == INVALID ? count_ : _i;
      ret |= Biggest;
    }
    if (_val < min_)
    {
      min_ = _val;
      min_idx_ = _i == INVALID ? count_ : _i;
      ret |= Smallest;
    }
    sum_ += _val;
    ++count_;
    return ret;
  }
  const ValueT& max() const { return max_; }
  const ValueT& min() const { return min_; }
  const size_t& max_idx() const { return max_idx_; }
  const size_t& min_idx() const { return min_idx_; }
  const ValueT& sum() const { return sum_; }
  const ValueT& average() const { return sum_ / count_; }
  const size_t count() { return count_; }
};

}//namespace Geo
