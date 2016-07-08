
#include "geom.hh"
#include "iterator.hh"

namespace Topo {

Geo::Point face_normal(Topo::Wrap<Topo::Type::FACE> _face)
{
  Iterator<Type::FACE, Type::VERTEX> fv_it;
  std::vector<Geo::Point> verts;
  for (auto vert : fv_it)
  {
    verts.emplace_back();
    vert->geom(verts.back());
  }
  auto poly_face = Geo::IPolygonalFace::make(verts.begin(), verts.end());
  return poly_face->normal();
}

Geo::Point coedge_direction(Topo::Wrap<Topo::Type::COEDGE> _coed)
{
  Geo::Segment seg;
  _coed->geom(seg);
  return seg[1] - seg[0];
}

}//namespace Topo