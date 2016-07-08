
#include "Boolean/priv.hh"
#include "Utils/statistics.hh"
#include "Topology/iterator.hh"
#include "Topology/split.hh"
#include "Geo/entity.hh"

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
  for (size_t i = 0; i < _ed_it.size(); ++i)
  {
    Topo::Wrap<Topo::Type::EDGE> edge = _ed_it.get(i);
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev;
    ev.reset(edge);
    for (size_t j = 0; j < _vert_it.size(); ++j)
    {
      Topo::Split<Topo::Type::EDGE>::Info spli;
      spli.vert_ = _vert_it.get(j);

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
      Geo::closest_point(seg, pt, &spli.clsst_pt_, &spli.t_, &spli.dist_);
      Utils::FindMax<double> max_tol(spli.vert_->tolerance());
      max_tol.add(edge->tolerance());
      if (spli.dist_ > max_tol())
        continue; // Vertex is too far.

      auto it = ed_splt_set_.lower_bound(edge);
      if (it == ed_splt_set_.end() || *it != edge)
        it = ed_splt_set_.emplace_hint(it, edge);
      it->add_point(spli);
    }
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
