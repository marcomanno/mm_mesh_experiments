#pragma once

#include <iostream>

namespace Utils {

template <typename Data> struct BinData
{
  BinData(const Data& _dat) : dat_(reinterpret_cast<const char*>(&_dat)) {}
  friend std::ostream& operator <<(std::ostream& _str, const BinData<Data>& _dat)
  {
    _str.write(_dat.dat_, sizeof(Data));
    return _str;
  }
  friend std::ostream& operator >>(std::istream& _str, BinData<Data>& _dat)
  {
    _str.read(_dat.dat_, sizeof(Data));
    return _str;
  }
private:
  const char* dat_;
};

}