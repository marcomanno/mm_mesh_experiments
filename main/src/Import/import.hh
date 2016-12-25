#pragma once

#include <Topology/topology.hh>

namespace IO {

Topo::Wrap<Topo::Type::BODY> load_obj(const char* _flnm);
bool save_obj(const char* _flnm, Topo::Wrap<Topo::Type::BODY>);
bool save_face(const Topo::E<Topo::Type::FACE>* _ptr, int _num, 
  const bool _split = true);

}//namespace Import