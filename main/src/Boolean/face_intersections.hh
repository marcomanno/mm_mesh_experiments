#pragma once

#include "priv.hh"
//#include "UtilsIndex.h"

#include <memory>
#include <map>
#include <vector>


namespace Boolean {

struct FaceVersus : public IFaceVersus
{
  virtual bool vertex_intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it);

  virtual bool edge_intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _edge_it);

  virtual bool face_intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_a,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_b);

  virtual const OverlapFces& overlap_faces() const
  {
    return overlap_faces_;
  }


private:
  struct FaceVertexInfo
  {
    std::shared_ptr<Geo::IPolygonalFace> poly_face_;
    std::vector<Topo::Wrap<Topo::Type::VERTEX>> new_vert_list_;
  };

  FaceVertexInfo& face_geom(const Topo::Wrap<Topo::Type::FACE>& _face);

  std::map<Topo::Wrap<Topo::Type::FACE>, FaceVertexInfo> f_vert_info_;

  OverlapFces overlap_faces_;
};

}//namespace Boolean
