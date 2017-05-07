#pragma once

#include <Topology/topology.hh>

namespace IO {

Topo::Wrap<Topo::Type::BODY> load_obj(const char* _flnm);
bool save_obj(const char* _flnm, Topo::Wrap<Topo::Type::BODY>);
bool save_face(const Topo::E<Topo::Type::FACE>* _ptr, int _num, 
  const bool _split = true);

void save_obj(const char* _flnm,
  const std::vector<Geo::Vector3>& _plgn,
  const std::vector<size_t>* _inds = nullptr);

}//namespace Import