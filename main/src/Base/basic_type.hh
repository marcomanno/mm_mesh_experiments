#pragma once

namespace Base {

template<class VauleTypeT>
class BasicType {
  VauleTypeT val_;
public:
  BasicType(const VauleTypeT _val = 0) : val_(_val) {}
  BasicType(const BasicType<VauleTypeT>& _bt) : 
    val_(_bt.val_) {}

  typedef VauleTypeT value_type;
  operator VauleTypeT() const { return val_; }
  operator VauleTypeT&() { return val_; }
  //modifiers
  BasicType& operator=(VauleTypeT v) { val_ = v; return *this; }
  BasicType& operator+=(VauleTypeT v) { val_ += v; return *this; }
  BasicType& operator-=(VauleTypeT v) { val_ -= v; return *this; }
  BasicType& operator*=(VauleTypeT v) { val_ *= val_; return *this; }
  BasicType& operator/=(VauleTypeT v) { val_ /= val_; return *this; }
  BasicType& operator%=(VauleTypeT v) { val_ %= val_; return *this; }
  BasicType& operator++() { ++val_; return *this; }
  BasicType& operator--() { --val_; return *this; }
  BasicType operator++(int) { return BasicType(val_++); }
  BasicType operator--(int) { return BasicType(val_--); }
  BasicType& operator&=(VauleTypeT v) { val_ &= v; return *this; }
  BasicType& operator|=(VauleTypeT v) { val_ |= v; return *this; }
  BasicType& operator^=(VauleTypeT v) { val_ ^= v; return *this; }
  BasicType& operator<<=(VauleTypeT v) { val_ <<= v; return *this; }
  BasicType& operator>>=(VauleTypeT v) { val_ >>= v; return *this; }

  //accessors
  BasicType operator+() const { return BasicType(+val_); }
  BasicType operator-() const { return BasicType(-val_); }
  BasicType operator!() const { return BasicType(!val_); }
  BasicType operator~() const { return BasicType(~val_); }

  //friends
  friend BasicType operator+(BasicType iw, BasicType v) { return iw += v; }
  friend BasicType operator+(BasicType iw, VauleTypeT v) { return iw += v; }
  friend BasicType operator+(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) += iw; }
  friend BasicType operator-(BasicType iw, BasicType v) { return iw -= v; }
  friend BasicType operator-(BasicType iw, VauleTypeT v) { return iw -= v; }
  friend BasicType operator-(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) -= iw; }
  friend BasicType operator*(BasicType iw, BasicType v) { return iw *= v; }
  friend BasicType operator*(BasicType iw, VauleTypeT v) { return iw *= v; }
  friend BasicType operator*(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) *= iw; }
  friend BasicType operator/(BasicType iw, BasicType v) { return iw /= v; }
  friend BasicType operator/(BasicType iw, VauleTypeT v) { return iw /= v; }
  friend BasicType operator/(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) /= iw; }
  friend BasicType operator%(BasicType iw, BasicType v) { return iw %= v; }
  friend BasicType operator%(BasicType iw, VauleTypeT v) { return iw %= v; }
  friend BasicType operator%(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) %= iw; }
  friend BasicType operator&(BasicType iw, BasicType v) { return iw &= v; }
  friend BasicType operator&(BasicType iw, VauleTypeT v) { return iw &= v; }
  friend BasicType operator&(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) &= iw; }
  friend BasicType operator|(BasicType iw, BasicType v) { return iw |= v; }
  friend BasicType operator|(BasicType iw, VauleTypeT v) { return iw |= v; }
  friend BasicType operator|(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) |= iw; }
  friend BasicType operator^(BasicType iw, BasicType v) { return iw ^= v; }
  friend BasicType operator^(BasicType iw, VauleTypeT v) { return iw ^= v; }
  friend BasicType operator^(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) ^= iw; }
  friend BasicType operator<<(BasicType iw, BasicType v) { return iw <<= v; }
  friend BasicType operator<<(BasicType iw, VauleTypeT v) { return iw <<= v; }
  friend BasicType operator<<(VauleTypeT v, BasicType iw) { return intWrapperImpl(v) <<= iw; }
  friend BasicType operator >> (BasicType iw, BasicType v) { return iw >>= v; }
  friend BasicType operator >> (BasicType iw, VauleTypeT v) { return iw >>= v; }
  friend BasicType operator >> (VauleTypeT v, BasicType iw) { return intWrapperImpl(v) >>= iw; }
};

}//namespace Base
