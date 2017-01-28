#pragma once

#include "priv.hh"

#include "Utils/index.hh"
#include "Utils/merger.hh"

#include <memory>
#include <map>
#include <set>
#include <vector>

namespace Boolean {

struct FaceEdgeInfo
{
  std::map<Topo::Wrap<Topo::Type::FACE>, std::vector<Utils::Index>> f_v_refs_;

  typedef double Parameter;
  typedef std::tuple<Utils::Index, Parameter> EdgeVertexReference;
  std::map<Topo::Wrap<Topo::Type::EDGE>, std::vector<EdgeVertexReference>> e_v_refs_;

  struct VertexReferences : public Utils::Mergiable
  {
    Utils::Index this_idx_;
    Geo::Point pt_;
    double tol_;
    Topo::Wrap<Topo::Type::VERTEX> vert_;
    std::vector<Geo::Point> mrg_list_;
    std::set<Topo::Wrap<Topo::Type::EDGE>> edge_refs_;
    std::set<Topo::Wrap<Topo::Type::FACE>> face_refs_;
    FaceEdgeInfo* owner_;

    bool equivalent(const VertexReferences& _oth) const;
    void merge(const VertexReferences& _oth);
  };
  std::vector<VertexReferences> vertices_refs_;

  void add(const Geo::Point& _pt, const double _t,
    const Topo::Wrap<Topo::Type::EDGE>& _edge,
    const Topo::Wrap<Topo::Type::FACE>& _face);

  void merge();

  void split_edges();
};

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

  virtual bool process_edge_intersections();

  virtual const OverlapFces& overlap_faces() const
  {
    return overlap_faces_;
  }


private:
  struct FaceVertexInfo
  {
    std::shared_ptr<Geo::IPolygonalFace> poly_face_;
    // Vertices coming from the intersection of the face with
    // vertices or edges.
    std::vector<Topo::Wrap<Topo::Type::VERTEX>> new_vert_list_;
  };

  FaceVertexInfo& face_geom(const Topo::Wrap<Topo::Type::FACE>& _face);

  std::map<Topo::Wrap<Topo::Type::FACE>, FaceVertexInfo> f_vert_info_;

  OverlapFces overlap_faces_;
  FaceEdgeInfo f_eds_info_;
};

}//namespace Boolean
