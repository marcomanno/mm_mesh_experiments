
#include "geom.hh"
#include "iterator.hh"
#include "Utils/error_handling.hh"

namespace Topo {

Geo::Point face_normal(Topo::Wrap<Topo::Type::FACE> _face)
{
  Iterator<Type::FACE, Type::LOOP> fl_it(_face);
  THROW_IF(fl_it.size() == 0, "Normal ofnot defined face");
  Iterator<Type::LOOP, Type::VERTEX> lv_it(*fl_it.begin());
  std::vector<Geo::Point> verts;
  for (auto vert : lv_it)
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
  const Topo::Wrap<Topo::Type::FACE>& _face, const Geo::Point& _pt)
{
  Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> fl_it(_face);
  Geo::PointInPolygon::Classification out_res = Geo::PointInPolygon::Classification::Outside;
  for (auto& loop : fl_it)
  {
    Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> fv_it(loop);
    std::vector<Geo::Point> polygon(fv_it.size());
    for (int i = 0; i < fv_it.size(); ++i)
      fv_it.get(i)->geom(polygon[i]);
    auto pt_cl = Geo::PointInPolygon::classify(polygon, _pt);
    if (pt_cl == Geo::PointInPolygon::Classification::On)
      return pt_cl;
    if (pt_cl == out_res)
      return Geo::PointInPolygon::Classification::Outside;
    out_res = Geo::PointInPolygon::Classification::Inside;
  }
  return Geo::PointInPolygon::Classification::Inside;
}

Geo::PointInPolygon::Classification classify(
  const VertexChain& _vert_ch, const Geo::Point& _pt)
{
  std::vector<Geo::Point> polygon(_vert_ch.size());
  for (int i = 0; i < _vert_ch.size(); ++i)
    _vert_ch[i].get()->geom(polygon[i]);
  return Geo::PointInPolygon::classify(polygon, _pt);
}
}//namespace PointInFace
}//namespace Topo