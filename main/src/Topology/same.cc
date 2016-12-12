
#include "same.hh"

namespace Topo {

bool same(Wrap<Type::FACE>& _face0, Wrap<Type::FACE>& _face1,
  bool* _reversed)
{
  if (_reversed) *_reversed = false;
  Iterator<Type::FACE, Type::VERTEX> f0_it(_face0);
  Iterator<Type::FACE, Type::VERTEX> f1_it(_face1);
  if (f0_it.size() != f1_it.size())
    return false;
  if (f0_it.size() == 0)
    return true;
  auto it1 = f1_it.begin();
  auto it0 = std::find(f0_it.begin(), f0_it.end(), *it1);
  if (it0 == f0_it.end())
    return false;
  auto it0r = it0;
  bool same_order = true, inverted_order = true;
  while (++it1 != f1_it.end() && (same_order || inverted_order))
  {
    if (same_order)
    {
      if (++it0 == f0_it.end())
        it0 = f0_it.begin();
      if (*it0 != *it1)
      {
        same_order = false;
        if (!inverted_order)
          return false;
      }
    }
    if (inverted_order)
    {
      if (it0r == f0_it.begin())
        it0r = f0_it.end();
      if (*--it0r != *it1)
      {
        inverted_order = false;
        if (!same_order)
          return false;
      }
    }
  }
  if (_reversed)
    *_reversed = inverted_order;
  return true;
}

}