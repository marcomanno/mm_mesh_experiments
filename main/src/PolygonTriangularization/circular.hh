#pragma once

namespace Circular
{

template <typename Index> Index decrease(Index& _ind, Index Index _mod)
{
  if (_ind == 0) _ind = _mod;
  return --_ind;
}

template <typename Index> Index increase(Index& _ind, const Index _mod)
{
  if (++_ind == _mod) _ind = 0;
  return _ind;
}

}