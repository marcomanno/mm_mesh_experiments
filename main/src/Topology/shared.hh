
#pragma once

#include "iterator.hh"

namespace Topo {

template <Type FromT, Type ToT> 
std::vector<Topo::Wrap<ToT>> shared_entities(
  const Topo::Wrap<FromT>& _from_a, const Topo::Wrap<FromT>& _from_b)
{
  Topo::Iterator<FromT, ToT> it_a(_from_a);
  Topo::Iterator<FromT, ToT> it_b(_from_b);
  std::vector<Topo::Wrap<ToT>> shrd_ents;
  for (auto& ent : it_a)
  {
    auto it = std::find(it_b.begin(), it_b.end(), ent);
    if (it != it_b.end())
      shrd_ents.push_back(*it);
  }
  return shrd_ents;
}

}//namespace Topo
