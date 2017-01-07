
#include "face_intersections.hh"
#include "Geo/kdtree.hh"
#include "Geo/pow.hh"

#include <set>

namespace Boolean {

bool FaceVersus::vertex_intersect(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it)
{
  Geo::KdTree<Topo::Wrap<Topo::Type::FACE>> kdtree_f;
  Geo::KdTree<Topo::Wrap<Topo::Type::VERTEX>> kdtree_v;

  kdtree_f.insert(_face_it.begin(), _face_it.end());
  kdtree_v.insert(_vert_it.begin(), _vert_it.end());
  kdtree_f.compute();
  kdtree_v.compute();

  std::vector<std::array<size_t, 2>> couples =
    Geo::find_kdtree_couples<Topo::Wrap<Topo::Type::FACE>, Topo::Wrap<Topo::Type::VERTEX>>(
      kdtree_f, kdtree_v);

  for (auto& couple : couples)
  {
    const auto& face = kdtree_f[couple[0]];
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(face);
    std::set<Topo::Wrap<Topo::Type::VERTEX>> face_verts(fv_it.begin(), fv_it.end());
    auto& face_info = face_geom(face);
    const auto& vert = kdtree_v[couple[1]];
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
  return true;
}

}