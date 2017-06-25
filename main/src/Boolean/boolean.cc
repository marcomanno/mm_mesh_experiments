
#include "boolean.hh"
#include "priv.hh"
#include <Topology/iterator.hh>
#include <Topology/impl.hh>

#define DEB_ON
#ifdef DEB_ON
#include <Import/import.hh>
#endif

#include <iostream>

namespace Boolean {

namespace {

template <Topo::Type typeT>
struct ItearatorCache
{
  Topo::Iterator<Topo::Type::BODY, typeT> ents_;
  Topo::Iterator<Topo::Type::BODY, typeT>& get(Topo::Wrap<Topo::Type::BODY>& _body)
  {
    if (ents_.size() == 0)
      ents_.reset(_body);
    return ents_;
  }
  void clear() { ents_.clear(); }
};

struct BodyInfo : public 
  ItearatorCache<Topo::Type::VERTEX>, 
  ItearatorCache<Topo::Type::EDGE>,
  ItearatorCache<Topo::Type::FACE>
{
  void init(Topo::Wrap<Topo::Type::BODY> _body)
  {
    body_ = _body;
  }

  template <Topo::Type typeT>
  Topo::Iterator<Topo::Type::BODY, typeT>& iterator()
  {
    return ItearatorCache<typeT>::get(body_);
  }

  void clear()
  {
    ItearatorCache<Topo::Type::VERTEX>::clear();
    ItearatorCache<Topo::Type::EDGE>::clear();
    ItearatorCache<Topo::Type::FACE>::clear();
  }

  Topo::Wrap<Topo::Type::BODY> body_;
};


struct Solver : public ISolver
{
  virtual void init(Topo::Wrap<Topo::Type::BODY> _body_a,
    Topo::Wrap<Topo::Type::BODY> _body_b)
  {
    bodies_[0].init(_body_a);
    bodies_[1].init(_body_b);
  }

  virtual Topo::Wrap<Topo::Type::BODY> compute(const Operation _op);

private:

  Topo::Wrap<Topo::Type::BODY> make_result();

  std::array<BodyInfo, 2> bodies_;
};

Topo::Wrap<Topo::Type::BODY> Solver::compute(const Operation _op)
{
  auto clean_up = [this]()
  {
#ifdef DEB_ON
    //IO::save_obj("pre_0.obj", bodies_[0].body_);
    //IO::save_obj("pre_1.obj", bodies_[1].body_);
#endif
    while (remove_degeneracies(bodies_[0].body_, bodies_[1].body_));
    bodies_[0].clear();
    bodies_[1].clear();
#ifdef DEB_ON
    static int n = 0;
    std::string str = "debug_";
    str += std::to_string(n++);
    IO::save_obj((str + "_0.obj").c_str(), bodies_[0].body_);
    IO::save_obj((str + "_1.obj").c_str(), bodies_[1].body_);
#endif
  };

  vertices_versus_vertices(
    bodies_[0].iterator<Topo::Type::VERTEX>(),
    bodies_[1].iterator<Topo::Type::VERTEX>());

  clean_up();

  auto vert_eds = IEdgesVersusVertices::make();
  vert_eds->intersect(
    bodies_[0].iterator<Topo::Type::VERTEX>(),
    bodies_[1].iterator<Topo::Type::EDGE>());

  vert_eds->intersect(
    bodies_[1].iterator<Topo::Type::VERTEX>(),
    bodies_[0].iterator<Topo::Type::EDGE>());

  vert_eds->split();

  clean_up();

  auto eds_eds = IEdgeVersusEdges::make();
  eds_eds->intersect(
    bodies_[0].iterator<Topo::Type::EDGE>(),
    bodies_[1].iterator<Topo::Type::EDGE>());
  eds_eds->split();

  clean_up();

  auto face_all = IFaceVersus::make();
  face_all->vertex_intersect(
    bodies_[0].iterator<Topo::Type::FACE>(),
    bodies_[1].iterator<Topo::Type::VERTEX>());

  face_all->vertex_intersect(
    bodies_[1].iterator<Topo::Type::FACE>(),
    bodies_[0].iterator<Topo::Type::VERTEX>());

  clean_up();

  face_all->edge_intersect(
    bodies_[0].iterator<Topo::Type::FACE>(),
    bodies_[1].iterator<Topo::Type::EDGE>());

  face_all->edge_intersect(
    bodies_[1].iterator<Topo::Type::FACE>(),
    bodies_[0].iterator<Topo::Type::EDGE>());

  face_all->process_edge_intersections();

  clean_up();

  face_all->face_intersect(
    bodies_[0].iterator<Topo::Type::FACE>(),
    bodies_[1].iterator<Topo::Type::FACE>());

  clean_up();

  auto selector = ISelection::make(_op);
  selector->select_overlap_faces(face_all->overlap_faces());
  selector->select_faces(bodies_[0].body_, bodies_[1].body_);

  return make_result();
}

// Move all faces to a new body.
Topo::Wrap<Topo::Type::BODY> Solver::make_result()
{
  Topo::Wrap<Topo::Type::BODY> new_body;
  auto new_body_data = new_body.make<Topo::EE<Topo::Type::BODY>>();
  for (int i = 0; i < 2; ++i)
  {
    auto ind = bodies_[i].body_->size(Topo::Direction::Down);
    while (ind-- > 0)
    {
      auto ent = bodies_[i].body_->get(Topo::Direction::Down, ind);
      new_body_data->insert_child(ent);
      bodies_[i].body_->remove_child(ind);
    }
  }
  return new_body;
}

}//namespace 

std::shared_ptr<ISolver> ISolver::make()
{
  return std::make_shared<Solver>();
}

}