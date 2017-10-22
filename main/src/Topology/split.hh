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

  void add_boundary(const VertexChain& _vert_chain)
  {
    boundary_chains_.emplace_back(std::move(_vert_chain));
  }

  void add_island(const VertexChain& _vert_chain)
  {
    island_chains_.emplace_back(std::move(_vert_chain));
  }

  void add_original_island(const VertexChain& _vert_chain)
  {
    original_island_chains_.emplace_back(std::move(_vert_chain));
  }

  // Add the current face loops as boundary or isles.
  struct LoopFilter
  {
    virtual bool operator()(const Topo::Wrap<Topo::Type::LOOP>&) const { return true; }
  };
  void use_face_loops(const LoopFilter& _loop_filter = LoopFilter());

  bool compute();

  const std::vector<Wrap<Type::FACE>>& new_faces() const { return new_faces_; }

  const Wrap<Type::FACE>& face() const { return face_; }

private:
  Wrap<Type::FACE> face_;
  std::vector<Wrap<Type::FACE>> new_faces_;
  std::vector<VertexChain> boundary_chains_;
  std::vector<VertexChain> island_chains_;
  std::vector<VertexChain> original_island_chains_;
};

bool split(const Wrap<Type::VERTEX>& _ed_start,
  const Wrap<Type::VERTEX>& _ed_end, Wrap<Type::VERTEX>& _ins_vert);

}//namespace Topology
