#pragma once

#include <Topology/Topology.hh>
#include "Utils/enum.hh"

#include <memory>

namespace Boolean {

MAKE_ENUM(Operation, UNION, INTERSECTION, DIFFERENCE, SPLIT, SPLITA, SPLITB)

struct ISolver
{
  virtual ~ISolver() {}
  virtual void init(Topo::Wrap<Topo::Type::BODY> _body_a, Topo::Wrap<Topo::Type::BODY> _body_b) = 0;
  virtual Topo::Wrap<Topo::Type::BODY> compute(const Operation _op) = 0;
  static std::shared_ptr<ISolver> make();
};


}