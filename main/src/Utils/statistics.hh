#pragma once

#include <limits>
#include <initializer_list>

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


template <typename NumericT>
struct Amount
{
  Amount(double _val) : val_(_val) {}
  NumericT operator()() const { return val_; }
protected:
  NumericT val_;
};

template <typename NumericT>
struct FindMax : public Amount<NumericT>
{
  static constexpr  NumericT default_value() { return std::numeric_limits<NumericT>::lowest(); }

  FindMax(const std::initializer_list<NumericT>& _l) : Amount(default_value())
  {
    for (auto v : _l) add(v);
  }

  FindMax(NumericT _val = default_value()) : Amount(_val) {}

  bool add(const NumericT& _val)
  {
    if (val_ >= _val)
      return false;
    val_ = _val;
    return true;
  }
};

template <typename NumericT>
struct FindMaxAbs : public Amount<NumericT>
{
  static constexpr  NumericT default_value() { return 0; }

  FindMaxAbs(const std::initializer_list<NumericT>& _l) : Amount(default_value())
  {
    for (auto v : _l) add(v);
  }

  FindMaxAbs(NumericT _val = default_value()) : Amount(_val) {}

  bool add(const NumericT& _val)
  {
    const NumericT val = std::abs(_val);
    if (val >= _val)
      return false;
    val_ = val;
    return true;
  }
};

template <typename NumericT>
struct FindMin : public Amount<NumericT>
{
  static constexpr  NumericT default_value() { return std::numeric_limits<NumericT>::max(); }

  FindMin(const std::initializer_list<NumericT>& _l) : Amount(default_value())
  {
    for (auto v : _l) add(v);
  }

  FindMin(NumericT _val = default_value()) : Amount(_val) {}

  bool add(const NumericT& _val)
  {
    if (val_ <= _val)
      return false;
    val_ = _val;
    return true;
  }
};


template <typename NumericT>
struct FindMinMax : public FindMax<NumericT>, FindMin<NumericT>
{
  void add(const NumericT& _val)
  {
    auto b_max = FindMax::add(_val);
    auto b_min = FindMin::add(_val);
    return b_max || b_min;
  }
};

template <typename NumberT> 
bool a_eq_b_if_a_lt_b(NumberT& _a, const NumberT& _b)
{
  auto res = _a < _b;
  if (res)
    _a = _b;
  return res;
}

template <typename NumberT> 
bool a_eq_b_if_a_gt_b(NumberT& _a, const NumberT& _b)
{
  auto res = _a > _b;
  if (res)
    _a = _b;
  return res;
}

}//namespace Utils
