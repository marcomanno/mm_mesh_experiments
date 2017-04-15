
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
    geom.poly_face_ = Geo::IPolygonalFace::make();

    Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> fl_it(_face);
    for (auto loop : fl_it)
    {
      Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> lv_it(loop);
      std::vector<Geo::Point> pts(lv_it.size());
      for (size_t i = 0; i < lv_it.size(); ++i)
        lv_it.get(i)->geom(pts[i]);
      geom.poly_face_->add_loop(pts.begin(), pts.end());
    }
    geom.poly_face_->compute();
  }
  return geom;
}

std::shared_ptr<IFaceVersus> IFaceVersus::make() { return std::make_shared<FaceVersus>(); }

}//namespace Boolean
