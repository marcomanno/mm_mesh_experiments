
#include "face_intersections.hh"
#include "Geo/pow.hh"

#include <set>

namespace Boolean {

bool FaceVersus::vertex_intersect(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it)
{
  for (auto& face : _face_it)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(face);
    std::set<Topo::Wrap<Topo::Type::VERTEX>> face_verts(fv_it.begin(), fv_it.end());
    auto& face_info = face_geom(face);
    for (auto& vert : _vert_it)
    {
      if (face_verts.find(vert) != face_verts.end())
        continue;
      Geo::Point pt, clsst_pt;
      double dist_sq;
      vert->geom(pt);
      if (!closest_point(*face_info.poly_face_, pt, &clsst_pt, &dist_sq))
        continue;
      if (dist_sq > Geo::sq(vert->tolerance()))
        continue;
      face_info.new_vert_list_.push_back(vert);
    }
  }
  return true;
}

}