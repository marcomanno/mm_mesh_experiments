
#include "face_intersections.hh"
#include "priv.hh"

#include "Topology/impl.hh"
#include "Geo/entity.hh"
#include "Geo/pow.hh"
#include "Geo/vector.hh"
#include "Utils/merger.hh"

#include<map>
#include<vector>
#include<set>

namespace Boolean {

FaceVersus::FaceVertexInfo& 
FaceVersus::face_geom(const Topo::Wrap<Topo::Type::FACE>& _face)
{
  auto& geom = f_vert_info_[_face];
  if (!geom.poly_face_)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(_face);
    std::vector<Geo::Point> pts(fv_it.size());
    for (size_t i = 0; i < fv_it.size(); ++i)
      fv_it.get(i)->geom(pts[i]);
    geom.poly_face_ = Geo::IPolygonalFace::make(pts.begin(), pts.end());
  }
  return geom;
}

std::shared_ptr<IFaceVersus> IFaceVersus::make() { return std::make_shared<FaceVersus>(); }

}//namespace Boolean
