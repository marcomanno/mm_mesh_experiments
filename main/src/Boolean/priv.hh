#pragma once

#include "boolean.hh"
#include "Topology/iterator.hh"

#include <memory>

namespace Boolean {

bool vertices_versus_vertices(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it_a,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it_b);

struct IEdgesVersusVertices
{
  virtual bool intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it_a,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_b) = 0;

  virtual bool split() = 0;

  static std::shared_ptr<IEdgesVersusVertices> make();
};

struct IEdgeVersusEdges
{
  virtual bool intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_a,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_b) = 0;

  virtual bool split() = 0;

  static std::shared_ptr<IEdgeVersusEdges> make();
};

typedef std::array<std::vector<Topo::Wrap<Topo::Type::FACE>>, 2> OverlapFces;

struct IFaceVersus
{
  virtual bool vertex_intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it) = 0;

  virtual bool edge_intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _edge_it) = 0;

  virtual bool face_intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_a,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_b) = 0;

  virtual const OverlapFces& overlap_faces() const = 0;

  static std::shared_ptr<IFaceVersus> make();
};

struct ISelection
{
  virtual void select_overlap_faces(const OverlapFces& _overlap_faces) = 0;

  virtual void select_faces(Topo::Wrap<Topo::Type::BODY>& _body_a,
    Topo::Wrap<Topo::Type::BODY>& _body_b) = 0;

  static std::shared_ptr<ISelection> make(Operation _bool_op);
};

// Merges faces and splits concave mesh faces in triangles or at least in convex pieces.
void simplify_and_correct(Topo::Wrap<Topo::Type::BODY>& _body);

bool remove_degeneracies(Topo::Wrap<Topo::Type::BODY>& _body);

}//namespace Boolean
