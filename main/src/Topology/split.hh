#pragma  once

#include "Topology.hh"
#include "Geo/entity.hh"

#include <vector>

namespace Topo {

template <Type TypeT> struct Split;

template <> struct Split<Type::EDGE>
{
  struct Info
  {
    Wrap<Type::VERTEX> vert_;
    Geo::Point clsst_pt_;
    double t_;
    double dist_sq_ = 0;
  };

  Split(Wrap<Type::EDGE>& _edge) : edge_(_edge) {}
  void add_point(const Info& inf_) const;
  void remove_duplicates();

  bool operator < (const Split<Type::EDGE>& _oth) const { return edge_ < _oth.edge_; }
  bool operator == (const Split<Type::EDGE>& _oth) const { return edge_ == _oth.edge_; }
  bool operator != (const Split<Type::EDGE>& _oth) const { return edge_ != _oth.edge_; }

  bool operator()() const;

private:
  Wrap<Type::EDGE> edge_;
  mutable std::vector<Info> split_pts_;
  mutable std::vector<Wrap<Type::EDGE>> split_edges_;
};

template <> struct Split<Type::FACE>
{
  Split(Wrap<Type::FACE>& _face) : face_(_face) {}

  bool operator()(VertexChains& _chains);

  const std::vector<Wrap<Type::FACE>>& new_faces() const { return new_faces_; }

  const Wrap<Type::FACE>& face() const { return face_; }

private:
  Wrap<Type::FACE> face_;
  std::vector<Wrap<Type::FACE>> new_faces_;
};

}//namespace Topology
