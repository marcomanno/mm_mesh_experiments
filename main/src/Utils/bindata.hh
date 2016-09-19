#pragma once

#include <iostream>

namespace Utils {

template <typename Data> struct BinData
{
  BinData(const Data& _dat) : dat_(_dat) {}
  friend std::ostream& operator <<(std::ostream& _str, const BinData<Data>& _dat)
  {
    _str.write(reinterpret_cast<const char*>(&_dat.dat_), sizeof(Data));
    return _str;
  }
  friend std::istream& operator >>(std::istream& _str, const BinData<Data>& _dat)
  {
    _str.read(const_cast<char*>(reinterpret_cast<const char*>(&_dat.dat_)), sizeof(Data));
    return _str;
  }
private:
  const Data& dat_;
};

}