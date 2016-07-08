
#pragma once

#include "Topology.hh"

namespace Topo {

template <Type FromT, Type ToT> struct Iterator
{
  Iterator();
  Iterator(const Wrap<FromT>& _from) : Iterator() { reset(_from); }
  void reset(const Wrap<FromT>& _from);
  ~Iterator();
  size_t size() const;
  Wrap<ToT> get(size_t _i) const;
  Wrap<ToT>* begin() const;
  Wrap<ToT>* end() const;
  void clear();
private:
  struct Impl;
  Impl* impl_;
};

}//namespace Topo
