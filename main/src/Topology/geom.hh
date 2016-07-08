#pragma once
#include "Geo/entity.hh"
#include "Geo/vector.hh"
#include "Topology/Topology.hh"

namespace Topo {

Geo::Point face_normal(Topo::Wrap<Topo::Type::FACE> _face);
Geo::Point coedge_direction(Topo::Wrap<Topo::Type::COEDGE> _coed);

}//namespace Topo