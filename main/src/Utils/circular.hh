#pragma once

namespace Utils
{

template <typename Index> Index decrease(const Index& _ind,const Index _mod)
{
  return _ind == 0 ? _mod - 1 : _ind - 1;
}

template <typename Index> Index increase(const Index& _ind, const Index _mod)
{
  return _ind == _mod - 1 ?  0 : _ind + 1;
}

}