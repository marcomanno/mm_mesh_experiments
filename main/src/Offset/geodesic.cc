#include "Geo\entity.hh"
#include "Geo\vector.hh"
#include "Topology\topology.hh"
#include "Topology\impl.hh"
#include "Topology\iterator.hh"

#if 1


#include <map>
#include <queue>
#include <set>

namespace Offset
{

struct EdgeDistance
{
  Topo::Wrap<Topo::Type::FACE> origin_face_; // Face used to
  double x_;
  double y_[2];
  Geo::Range<1> param_range_;                // Portion of edges
  Geo::Range<1> distance_range_;             // range of distances
  std::array<EdgeDistance*, 2> children_ = {nullptr};
  EdgeDistance* parent_ = nullptr;
};

struct GeodesicDistance
{
  Geo::Point origin_;
  using EdgeDistances = std::multimap<Topo::Wrap<Topo::Type::EDGE>, EdgeDistance>;
  using EdgeAndDistance = EdgeDistances::value_type;
  // First smaller distances. Priority queue process first big elements.
  struct CompareEdgeDistancePtr
  {
    constexpr bool operator()(const EdgeAndDistance* _a, const EdgeAndDistance* _b) const
    {
      return _a->second.distance_range_[0][1] > _b->second.distance_range_[0][1];
    }
  };

  EdgeDistances edge_distances_;
  std::priority_queue<const EdgeAndDistance*, std::vector<const EdgeAndDistance*>,
    CompareEdgeDistancePtr> priority_queue_;

  bool compute(Topo::Wrap<Topo::Type::VERTEX> _v);
private:
  void advance(const EdgeAndDistance* _ed_span, 
               const Topo::Wrap<Topo::Type::FACE>& _f);
};

bool GeodesicDistance::compute(Topo::Wrap<Topo::Type::VERTEX> _v)
{
  _v->geom(origin_);
  Topo::Iterator<Topo::Type::VERTEX, Topo::Type::EDGE> vert_it(_v);
  for (auto e : vert_it)
  {
    EdgeDistance ed_dist;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ed_vert_it(e);
    auto v0 = ed_vert_it.get(0), v1 = ed_vert_it.get(1);
    if (v1 == _v)
      std::swap(v0, v1);
    Geo::Point pt;
    v1->geom(pt);
    ed_dist.x_ = 0;
    ed_dist.y_[0] = 0;
    ed_dist.y_[1] = Geo::length(pt - origin_);
    ed_dist.param_range_ = { 0, 1 };
    ed_dist.distance_range_ = { 0, ed_dist.y_[1] };
    edge_distances_.emplace(e, ed_dist);
  }
  Topo::Iterator<Topo::Type::VERTEX, Topo::Type::FACE> face_it(_v);
  for (auto f : face_it)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::EDGE> ed_it(f);
    Topo::Wrap<Topo::Type::EDGE> curr_ed;
    for (auto e : ed_it)
    {
      auto it = edge_distances_.equal_range(e);
      if (it.first == it.second)
      {
        curr_ed = e;
        break;
      }
    }
    if (curr_ed.get() == nullptr)
      continue;
    EdgeDistance ed_dist;
    ed_dist.origin_face_ = f;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ed_vert(curr_ed);
    Geo::Segment seg;
    ed_vert.get(0)->geom(seg[0]);
    ed_vert.get(1)->geom(seg[1]);
    double dist_sq, t;
    Geo::Vector3 proj;
    if (!Gen::closest_point<double, 3, true>(seg, origin_, &proj, &t, &dist_sq))
      return false;

    ed_dist.x_ = sqrt(dist_sq);
    ed_dist.y_[0] = Geo::length(seg[0] - proj);
    ed_dist.y_[1] = Geo::length(seg[1] - proj);
    if (t > 0 && t < 1)
    {
      ed_dist.y_[1] *= -1;
      ed_dist.distance_range_ += Geo::Vector<double, 1>({ Geo::length(proj - origin_) });
    }
    for (const auto& pt : seg)
      ed_dist.distance_range_ += Geo::Vector<double, 1>({ Geo::length(pt - origin_) });
    ed_dist.param_range_ = { 0, 1 };
    auto it = edge_distances_.emplace(curr_ed, ed_dist);
    priority_queue_.push(&(*it));
  }
  while (!priority_queue_.empty())
  {
    auto edge_span = priority_queue_.top();
    priority_queue_.pop();
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> fe_it(edge_span->first);
    for (auto f : fe_it)
    {
      if (f != edge_span->second.origin_face_)
        advance(edge_span, f);
    }
  }
  return true;
}

void GeodesicDistance::advance(
  const EdgeAndDistance* _ed_span, const Topo::Wrap<Topo::Type::FACE>& _f)
{
  _ed_span; _f;
}

} // namespace Offset

#endif