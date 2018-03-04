#include "Geo/entity.hh"
#include "Geo/polynomial_solver.hh"
#include "Topology/topology.hh"
#include "Topology/impl.hh"
#include "Topology/iterator.hh"
#include "Utils/error_handling.hh"

#include <bitset>
#include <functional>

#if 1


#include <map>
#include <queue>
#include <set>

namespace Offset
{
struct EdgeDistance;
using EdgeDistances = std::multimap<Topo::Wrap<Topo::Type::EDGE>, EdgeDistance>;
using EdgeAndDistance = EdgeDistances::value_type;

struct EdgeDistance
{
  Geo::VectorD3 ed_vec_;
  Topo::Wrap<Topo::Type::FACE> origin_face_; // Face used to get there
  double x_;
  double y_ed_[2];
  Geo::Interval<double> param_range_;  // Portion of edges
  bool skip_ = false;

  void init()
  {
    y_[0] = y_ed_[0] + (y_ed_[1] - y_ed_[0]) * param_range_[0];
    y_[1] = y_ed_[0] + (y_ed_[1] - y_ed_[0]) * param_range_[1];
    distance_range_.set_empty();
    distance_range_.add(sqrt(Geo::sq(x_) + Geo::sq(y_[0])));
    distance_range_.add(sqrt(Geo::sq(x_) + Geo::sq(y_[1])));
    if ((y_[0] > 0) != (y_[1] > 0))
      distance_range_.add(x_);
  }

  double y_[2];
  Geo::Interval<double> distance_range_;             // range of distances
  std::array<EdgeAndDistance*, 2> children_ = {nullptr};
  const EdgeAndDistance* parent_ = nullptr;
  Geo::VectorD3 get_distance_function()
  {
    // x_^2 + (y0 + t * (y1 - y0))^2 = 
    // = (x^2 + y0^2) + t * (2 * y0 * (y1 - y0)) + t^2 * (y1 - y0)^2
    auto dy = y_ed_[1] - y_ed_[0];
    return Geo::VectorD3{ { Geo::sq(x_) + Geo::sq(y_ed_[0]), 2 * y_ed_[0] * dy, Geo::sq(dy)} };
  }
};

struct GeodesicDistance
{
  Geo::Point origin_;
  // First smaller distances. Priority queue process first big elements.
  struct CompareEdgeDistancePtr
  {
    bool operator()(const EdgeAndDistance* _a, const EdgeAndDistance* _b) const
    {
      return _a->second.distance_range_[1] > _b->second.distance_range_[1];
    }
  };

  EdgeDistances edge_distances_;
  std::priority_queue<const EdgeAndDistance*, std::vector<const EdgeAndDistance*>,
    CompareEdgeDistancePtr> priority_queue_;

  bool compute(Topo::Wrap<Topo::Type::VERTEX> _v);
private:
  void advance(const EdgeAndDistance* _ed_span, 
               const Topo::Wrap<Topo::Type::FACE>& _f);
  void merge(Topo::Wrap<Topo::Type::EDGE> _ed,
             EdgeDistance& _ed_dist);

  void insert_element(const Topo::Wrap<Topo::Type::EDGE> _ed,
                      EdgeDistance& _ed_dist)
  {
    auto it = edge_distances_.emplace(_ed, _ed_dist);
    priority_queue_.push(&(*it));
  }
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
    ed_dist.ed_vec_ = pt - origin_;
    ed_dist.x_ = 0;
    ed_dist.y_ed_[0] = 0;
    ed_dist.y_ed_[1] = Geo::length(pt - origin_);
    ed_dist.param_range_ = { 0, 1 };
    ed_dist.init();
    insert_element(e, ed_dist);
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
    Geo::VectorD3 proj;
    if (!Gen::closest_point<double, 3, true>(seg, origin_, &proj, &t, &dist_sq))
      return false;

    ed_dist.x_ = sqrt(dist_sq);
    ed_dist.y_ed_[0] = Geo::length(seg[0] - proj);
    ed_dist.y_ed_[1] = Geo::length(seg[1] - proj);
    ed_dist.param_range_ = { 0, 1 };
    ed_dist.init();
    insert_element(curr_ed, ed_dist);
  }
  while (!priority_queue_.empty())
  {
    auto edge_span = priority_queue_.top();
    if (edge_span->second.skip_)
      continue;
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
  Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_ev_pa(_ed_span->first);
  Topo::Wrap<Topo::Type::VERTEX> verts[2] = { it_ev_pa.get(0), it_ev_pa.get(1) };
  Geo::VectorD3 p_parent[2];
  Geo::iterate_forw<2>::eval([&verts, &p_parent](int _i) { verts[_i]->geom(p_parent[_i]); });
  Geo::VectorD3 v_parent = p_parent[1] - p_parent[0];
  Topo::Iterator<Topo::Type::FACE, Topo::Type::EDGE> it_fe(_f);
  struct OtherEdge
  {
    Topo::Wrap<Topo::Type::EDGE> ed_;
    bool inv_;
    Geo::VectorD3 v_;
  };
  OtherEdge oth_eds[2];
  for (auto& fe : it_fe)
  {
    if (fe == _ed_span->first)
      continue;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_ev(fe);
    bool idx = it_ev.get(1) == verts[1] || it_ev.get(0) == verts[1];
    auto inv = it_ev.get(0) == verts[idx];
    oth_eds[idx].ed_ = fe;
    oth_eds[idx].inv_ = inv;
    Geo::VectorD3 p[2];
    Geo::iterate_forw<2>::eval([&it_ev, &p](int _i) { it_ev.get(_i)->geom(p[_i]); });
    oth_eds[idx].v_ = p[1] - p[0];
    auto v0 = v_parent;
    if (idx) v0 *= -1.;
    v0 /= Geo::length(v0);
    auto v1 = oth_eds[idx].v_;
    if (inv) v1 *= -1.;
    auto dx = v0 * v1;
    auto dy = Geo::length(v0 % v1);
    auto len = std::hypot(dx, dy);
    auto sin_al = dy / len;
    auto cos_al = dx / len;
    EdgeDistance new_edd;
    Gen::Segment<double, 2> v2 = { {
      { _ed_span->second.x_, _ed_span->second.y_ed_[idx] },
      { _ed_span->second.x_ + dx, _ed_span->second.y_ed_[idx] + dy } } };
    if (idx)
      std::swap(v2[0], v2[1]);
    Gen::Segment<double, 2> seg0{ { { 0. }, v2[1] } };
    Gen::Segment<double, 2> span_seg{ {
      {_ed_span->second.x_, _ed_span->second.y_[0]},
      {_ed_span->second.x_, _ed_span->second.y_[1] }} };

    double pars[2];
    if (!Gen::closest_point<double, 2>(seg0, span_seg, nullptr, pars))
      continue;
    Gen::Segment<double, 2> w2;
    for (int i = 0; i < 2; ++i)
      w2[i] = { v2[i][0] * cos_al + v2[i][1] * sin_al, v2[i][1] * cos_al - v2[i][0] * sin_al };
    new_edd.x_ = v2[0][0];
    new_edd.y_ed_[0] = w2[0][1];
    new_edd.y_ed_[1] = w2[1][1];
    Geo::Interval<double> tmp;
    tmp.set(!idx, pars[1]);
    tmp.set(idx, double(idx));
    auto par_range = _ed_span->second.param_range_ * tmp;
    if (par_range.empty())
      continue;
    Gen::Segment<double, 2> l0{ { { 0. },{} } };

    for (auto b : { false, true })
    {
      double par;
      Geo::Vector<double, 2> pts[2];
      if (par_range[b] == tmp[b])
      {
        par = { static_cast<double>(b) };
        pts[1] = v2[b];
      }
      else
      {
        auto pt = Gen::evaluate<double, 2>(span_seg, par_range[b]);
        Gen::Segment<double, 2> proj{ { { 0. }, pt } };
        double t[2];
        Gen::closest_point<double, 2, true>(proj, v2, pts, t);
        par = t[1];
      }
      new_edd.param_range_.set(b, par);
    }
    new_edd.init();
    merge(fe, new_edd);
  }
  THROW_IF(!oth_eds[0].ed_.get() || !oth_eds[1].ed_.get(), "");
}

static std::bitset<2> intersect(EdgeDistance& _a, EdgeDistance& _b)
{
  auto inters_par_range = _a.param_range_ * _b.param_range_;
  if (inters_par_range.empty())
    return 0;
  auto poly_a = _a.get_distance_function();
  auto poly_b = _b.get_distance_function();
  THROW_IF(fabs(poly_a[2] - poly_b[2]) > 1e-8, "x^3 Coefficeient is notzero");
  auto poly = poly_a - poly_b;
  if (poly[1] == 0)
  {
    if (poly[2] > 0)
    {
      _a.param_range_.set_empty();
      return 0;
    }
    else
    {
      _b.param_range_.set_empty();
      return 1;
    }
  }
  // 
  auto root = -poly[0] / poly[1];
  Geo::Interval<double> near_a(Geo::Interval<double>::min(), root);
  Geo::Interval<double> near_b(root, Geo::Interval<double>::min());
  if (poly[1] < 0)
    std::swap(near_a, near_b);

  std::bitset<2> res;
  Geo::Interval<double> extra;
  res[0] = _a.param_range_.subtract(near_b, extra);
  res[1] = _b.param_range_.subtract(near_a, extra);
  return res;
}

void GeodesicDistance::merge(Topo::Wrap<Topo::Type::EDGE> _ed,
                             EdgeDistance& _ed_dist)
{
  auto range = edge_distances_.equal_range(_ed);
  for (auto it = range.first; it != range.second && !_ed_dist.param_range_.empty();)
  {
    if (it->second.skip_)
      continue;
    auto int_res = intersect(_ed_dist, it->second);
    if (!int_res.any())
      continue;
    if (it->second.param_range_.empty())
    {
      std::function<void(EdgeAndDistance* _ed_dist)> skip_children =
      [&](EdgeAndDistance* _ed_dist)
      {
        _ed_dist->second.skip_ = true;
        for (auto child : _ed_dist->second.children_)
          skip_children(child);
      };
      skip_children(&*(it));
    }
    if (_ed_dist.param_range_.empty())
      break;
  }
  if (!_ed_dist.param_range_.empty())
  {
    _ed_dist.init();
    insert_element(_ed, _ed_dist);
  }
}

} // namespace Offset



#endif