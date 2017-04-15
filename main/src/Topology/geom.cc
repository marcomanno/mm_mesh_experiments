
#include "geom.hh"
#include "iterator.hh"

namespace Topo {

Geo::Point face_normal(Topo::Wrap<Topo::Type::FACE> _face)
{
  Iterator<Type::FACE, Type::VERTEX> fv_it(_face);
  std::vector<Geo::Point> verts;
  for (auto vert : fv_it)
  {
    verts.emplace_back();
    vert->geom(verts.back());
  }
  auto poly_face = Geo::IPolygonalFace::make();
  poly_face->add_loop(verts.begin(), verts.end());
  poly_face->compute();
  return poly_face->normal();
}

Geo::Point coedge_direction(Topo::Wrap<Topo::Type::COEDGE> _coed)
{
  Geo::Segment seg;
  _coed->geom(seg);
  return seg[1] - seg[0];
}

namespace PointInFace {
Geo::PointInPolygon::Classification classify(
  Topo::Wrap<Topo::Type::FACE> _face, const Geo::Point& _pt)
{
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(_face);
  std::vector<Geo::Point> polygon(fv_it.size());
  for (int i = 0; i < fv_it.size(); ++i)
    fv_it.get(i)->geom(polygon[i]);
  return Geo::PointInPolygon::classify(polygon, _pt);
}

}//namespace PointInFace
}//namespace Topo