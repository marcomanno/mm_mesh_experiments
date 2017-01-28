
#include "priv.hh"
#include "Geo/entity.hh"
#include "Geo/kdtree.hh"
#include "Topology/iterator.hh"
#include "Topology/split.hh"
#include "Utils/statistics.hh"


#include <set>

namespace Boolean {

namespace {
struct EdgesVersusVertices : public IEdgesVersusVertices
{
  virtual bool intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it_a,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_b);
  virtual bool split();
private:
  std::set<Topo::Split<Topo::Type::EDGE>> ed_splt_set_;
};


bool EdgesVersusVertices::intersect(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it)
{
  Geo::KdTree<Topo::Wrap<Topo::Type::EDGE>> kdtree_e;
  Geo::KdTree<Topo::Wrap<Topo::Type::VERTEX>> kdtree_v;

  kdtree_e.insert(_ed_it.begin(), _ed_it.end());
  kdtree_v.insert(_vert_it.begin(), _vert_it.end());
  kdtree_e.compute();
  kdtree_v.compute();

  std::vector<std::array<size_t, 2>> couples =
    Geo::find_kdtree_couples<Topo::Wrap<Topo::Type::EDGE>, Topo::Wrap<Topo::Type::VERTEX>>(
      kdtree_e, kdtree_v);

  for (const auto& couple : couples)
  {
    Topo::Wrap<Topo::Type::EDGE> edge = kdtree_e[couple[0]];
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev(edge);

    Topo::Split<Topo::Type::EDGE>::Info spli;
    spli.vert_ = kdtree_v[couple[1]];

    bool found = false;
    for (auto k = ev.size(); k-- > 0; )
    {
      if (ev.get(k) == spli.vert_)
      {
        found = true;
        break;
      }
    }
    if (found)
      continue;  // The vertex is already on the edge.

    Geo::Point pt;
    spli.vert_->geom(pt);
    Geo::Segment seg;
    edge->geom(seg);
    Geo::closest_point(seg, pt, &spli.clsst_pt_, &spli.t_, &spli.dist_sq_);
    Utils::FindMax<double> max_tol(
    { spli.vert_->tolerance(), Geo::epsilon(pt), edge->tolerance() });
    if (spli.dist_sq_ > Geo::sq(max_tol()))
      continue; // Vertex is too far.

    auto it = ed_splt_set_.lower_bound(edge);
    if (it == ed_splt_set_.end() || *it != edge)
      it = ed_splt_set_.emplace_hint(it, edge);
    it->add_point(spli);
  }
  return true;
}

bool EdgesVersusVertices::split()
{
  for (auto split_op : ed_splt_set_)
    split_op();
  return true;
}


}//namespace

std::shared_ptr<IEdgesVersusVertices> IEdgesVersusVertices::make()
{
  return std::make_shared<EdgesVersusVertices>();
}

}//namespace Boolean
