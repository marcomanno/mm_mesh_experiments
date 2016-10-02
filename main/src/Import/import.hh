#pragma once

#include <Topology/topology.hh>

namespace Import {

Topo::Wrap<Topo::Type::BODY> load_obj(const char* _flnm);
bool save_obj(const char* _flnm, Topo::Wrap<Topo::Type::BODY>);

}//namespace Import