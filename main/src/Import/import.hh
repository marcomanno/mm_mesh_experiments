#pragma once

#include <Topology/topology.hh>

namespace IO {

Topo::Wrap<Topo::Type::BODY> load_obj(const char* _flnm);
bool save_obj(const char* _flnm, Topo::Wrap<Topo::Type::BODY>, bool _split = true);
bool save_face(const Topo::E<Topo::Type::FACE>* _ptr, int _num,
               const bool _split = true);
bool save_face(const Topo::E<Topo::Type::FACE>* _ptr, const char* _flnm,
               const bool _split);

void save_obj(const char* _flnm,
  const std::vector<Geo::VectorD3>& _plgn,
  const std::vector<size_t>* _inds = nullptr);

struct ISaver
{
  virtual void add_face(const Topo::Wrap<Topo::Type::FACE>& _f) = 0;
  virtual void add_edge(const Topo::Wrap<Topo::Type::EDGE>& _e) = 0;
  virtual bool compute(const char* _flnm) = 0;
  static std::shared_ptr<ISaver> make();
};

}//namespace Import