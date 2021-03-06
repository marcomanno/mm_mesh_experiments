#include "geodesic.hh"
#include "Geo/entity.hh"
#include "Geo/polynomial_solver.hh"
#include "Topology/topology.hh"
#include "Topology/impl.hh"
#include "Topology/iterator.hh"
#include "Utils/error_handling.hh"

#include <boost/math/tools/roots.hpp>

#include <bitset>
#include <functional>
#include <fstream>


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
  EdgeDistance()
  {
    static int id_seq = 0;
    id_ = id_seq++;
  }
  ~EdgeDistance()
  {}

  void init(const Topo::Wrap<Topo::Type::FACE>& _f,
            const EdgeAndDistance* _parent)
  {
    if (x_ < 0)
    {
      x_ *= -1;
      y_ed_[0] *= -1;
      y_ed_[1] *= -1;
    }
    update_parameter();
    origin_face_ = _f;
    parent_ = _parent;
  }

  void update_parameter()
  {
    y_[0] = y_ed_[0] + (y_ed_[1] - y_ed_[0]) * param_range_[0];
    y_[1] = y_ed_[0] + (y_ed_[1] - y_ed_[0]) * param_range_[1];
    distance_range_.set_empty();
    distance_range_.add(sqrt(Geo::sq(x_) + Geo::sq(y_[0])) + dist0_);
    distance_range_.add(sqrt(Geo::sq(x_) + Geo::sq(y_[1])) + dist0_);
    if ((y_[0] > 0) != (y_[1] > 0))
      distance_range_.add(x_ + dist0_);
  }

  double get_distance(double _par,  double* _first_der = nullptr) const
  {
    auto dy = y_ed_[1] - y_ed_[0];
    auto y_part = y_ed_[0] + _par * dy;
    auto dist = sqrt(Geo::sq(x_) + Geo::sq(y_part));
    if (_first_der != nullptr)
      *_first_der = dy * y_part / dist;
    return dist0_ + dist;
  }

  size_t parameters(const double _dist, std::array<double, 2>& _pars)
  {
    if (!distance_range_.contain_close(_dist))
      return 0;
    auto y = std::sqrt(Geo::sq(_dist - dist0_) - Geo::sq(x_));
    size_t sol_nmbr = 0;
    auto get_solution = [this, &sol_nmbr, &_pars](double _y)
    {
      _pars[sol_nmbr] = (_y - y_ed_[0]) / (y_ed_[1] - y_ed_[0]);
      if (_pars[sol_nmbr] >= 0 && _pars[sol_nmbr] < 1)
        ++sol_nmbr;
    };
    get_solution(y);
    get_solution(-y);
    return sol_nmbr;
  }

  enum class Status { Keep = 0, Skip = 1, Remove = 2 };

  int id_;
  Topo::Wrap<Topo::Type::FACE> origin_face_; // Face used to get there
  double x_;
  double y_ed_[2];
  double dist0_ = 0;
  Geo::Interval<double> param_range_;  // Portion of edges
  Status status_ = Status::Keep;
  double y_[2];
  Geo::Interval<double> distance_range_;             // range of distances
  std::array<EdgeAndDistance*, 2> children_ = {nullptr};
  const EdgeAndDistance* parent_ = nullptr;
};

void check(const EdgeAndDistance& _edd, const Geo::Point& _orig)
{
  auto t = _edd.second.param_range_.interpolate(0.5);
  Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev(_edd.first);
  Geo::Point pt, pt1;
  ev.get(0)->geom(pt);
  ev.get(1)->geom(pt1);
  pt = Geo::interpolate(pt, pt1, t);
  auto dist = Geo::angle(_orig, pt) * 5;
  if (!_edd.second.distance_range_.contain_close(dist)) dist; // std::cout << "error";
}

// Return 0 if _apr is a root, else 
// < 0 ==> _a < _b
// > 0 ==> _b < _a 

int find_minimum(const EdgeDistance& _a, const EdgeDistance& _b,
                 double& _par, Geo::Interval<double> _range)
{
  auto func = [&_a, &_b](const double& _t)
  {
    double der_a, der_b;
    auto val_a = _a.get_distance(_t, &der_a);
    auto val_b = _b.get_distance(_t, &der_b);
    return std::make_tuple(val_a - val_b, der_a - der_b);
  };
  boost::uintmax_t max_iter = 20;
  _par = boost::math::tools::newton_raphson_iterate(
    func, (_range[0] + _range[1]) / 2, _range[0], _range[1],
    std::numeric_limits<double>::digits - 4, max_iter);
  if (max_iter < 20)
    return 0;
  return std::get<0>(func(_range.interpolate(0.5))) > 0 ? 1 : -1;
}

struct GeodesicDistance: public IGeodesic
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

  bool compute(const Topo::Wrap<Topo::Type::VERTEX>& _v) override;

  bool find_graph(
    double _dist,
    std::vector<Geo::VectorD3>& _pts,
    std::vector<std::array<size_t, 2>>& _inds) override;

private:
  void advance(const EdgeAndDistance* _ed_span, 
               const Topo::Wrap<Topo::Type::FACE>& _f);
  void merge(Topo::Wrap<Topo::Type::EDGE> _ed,
             EdgeDistance& _ed_dist);

  void insert_element(const Topo::Wrap<Topo::Type::EDGE> _ed,
                      EdgeDistance& _ed_dist)
  {
    auto it = edge_distances_.emplace(_ed, _ed_dist);
    check(*it, origin_);
    priority_queue_.push(&(*it));
  }
};

std::shared_ptr<IGeodesic> IGeodesic::make()
{
  return std::make_shared<GeodesicDistance>();
}


bool GeodesicDistance::compute(const Topo::Wrap<Topo::Type::VERTEX>& _v)
{
  _v->geom(origin_);
  Topo::Iterator<Topo::Type::VERTEX, Topo::Type::EDGE> vert_it(_v);
  for (auto e : vert_it)
  {
    EdgeDistance ed_dist;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ed_vert_it(e);
    auto v1 = ed_vert_it.get(1);
    auto inv = v1 == _v;
    if (inv)
      v1 = ed_vert_it.get(0);
    Geo::Point pt;
    v1->geom(pt);
    ed_dist.x_ = 0;
    ed_dist.y_ed_[0] = 0;
    ed_dist.y_ed_[1] = Geo::length(pt - origin_);
    if (inv)
      std::swap(ed_dist.y_ed_[0], ed_dist.y_ed_[1]);
    ed_dist.param_range_ = { 0, 1 };
    ed_dist.init(Topo::Wrap<Topo::Type::FACE>(), nullptr);
    ed_dist.status_ = EdgeDistance::Status::Skip;
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
    if (t < 1 && t > 0)
      ed_dist.y_ed_[ed_dist.y_ed_[1] < ed_dist.y_ed_[0]] *= -1;
    ed_dist.param_range_ = { 0, 1 };
    ed_dist.init(f, nullptr);
    insert_element(curr_ed, ed_dist);
  }
  while (!priority_queue_.empty())
  {
    auto edge_span = priority_queue_.top();
    priority_queue_.pop();
    if (edge_span->second.status_ != EdgeDistance::Status::Keep)
      continue;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> fe_it(edge_span->first);
    for (auto f : fe_it)
    {
      if (f != edge_span->second.origin_face_)
        advance(edge_span, f);
    }
  }
  for (auto it = edge_distances_.begin(); it != edge_distances_.end();)
  {
    if (it->second.status_ == EdgeDistance::Status::Remove)
      it = edge_distances_.erase(it);
    else
      ++it;
  }
  std::ofstream outf("c:/t/out.txt");
  outf << "V0 V1 Id Px Py Pz Qx Qy Qz x y0 y1 interval0 interval1 distance0  distance1 dist0 par_id \n";
  for (auto& es : edge_distances_)
  {
    Geo::Segment seg;
    es.first->geom(seg);
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ed_it(es.first);
    outf << ed_it.get(0)->id() << " " << ed_it.get(1)->id() << " ";
    outf << es.second.id_ << " " << seg[0] << " " << seg[1] << " ";
    outf << es.second.x_ << " ";
    outf << es.second.y_ed_[0] << " " << es.second.y_ed_[1] << " ";
    outf << es.second.param_range_[0] << " " << es.second.param_range_[1] << " ";
    outf << es.second.distance_range_[0] << " " << es.second.distance_range_[1] << " ";
    outf << es.second.dist0_ << " ";
    if (es.second.parent_ == nullptr)
      outf << -1;
    else
      outf << es.second.parent_->second.id_;
    outf  << std::endl;
  }
  outf << "SIZE = " << edge_distances_.size() << "\n";
  return true;
}

void GeodesicDistance::advance(
  const EdgeAndDistance* _parent, const Topo::Wrap<Topo::Type::FACE>& _f)
{
  Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_ev_pa(_parent->first);
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
  Gen::Segment<double, 2> parent_seg{ {
    { _parent->second.x_, _parent->second.y_[0] },
    { _parent->second.x_, _parent->second.y_[1] } } };
  Gen::Segment<double, 2> parent_trace_segs[2];
  for (int i = 0; i < 2; ++i)
  {
    parent_trace_segs[i][0] = {};
    parent_trace_segs[i][1] = parent_seg[i];
  }

  OtherEdge oth_eds[2];
  auto dy_par = _parent->second.y_ed_[1] - _parent->second.y_ed_[0];
  for (auto& fe : it_fe)
  {
    if (fe == _parent->first)
      continue;
    EdgeDistance new_edd;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_ev(fe);
    bool idx = it_ev.get(1) == verts[1] || it_ev.get(0) == verts[1];
    auto inv = it_ev.get(1) == verts[idx];
    oth_eds[idx].ed_ = fe;
    oth_eds[idx].inv_ = inv;
    Geo::VectorD3 p[2];
    Geo::iterate_forw<2>::eval([&it_ev, &p](int _i) { it_ev.get(_i)->geom(p[_i]); });
    auto v0 = v_parent;
    if (dy_par < 0) v0 *= -1.;
    v0 /= Geo::length(v0);
    auto v1 = oth_eds[idx].v_ = p[1] - p[0];
    if (inv) v1 *= -1.;
    auto dy = v0 * v1;
    //if (dy * dy_par < 0)
    //  continue;
    auto dx = Geo::length(v0 % v1);
    Gen::Segment<double, 2> v2 = { {
      { _parent->second.x_, _parent->second.y_ed_[idx] },
      { _parent->second.x_ + dx, _parent->second.y_ed_[idx] + dy } } };
    if (inv)
      std::swap(v2[0], v2[1]);
    auto dist_3d = Geo::length(p[0] - p[1]);

    auto compute_x_y_ed = [dist_3d](const Gen::Segment<double, 2>& _v, EdgeDistance& _new_ed)
    {
      Geo::VectorD2 proj;
      double dist_sq, par;
      Gen::closest_point<double, 2, true>(_v, Geo::VectorD2{},
                                          &proj, &par, &dist_sq);
      _new_ed.x_ = sqrt(dist_sq);
      _new_ed.y_ed_[0] = Geo::length(_v[0] - proj);
      _new_ed.y_ed_[1] = Geo::length(_v[1] - proj);
      if (par > 0 && par < 1)
        _new_ed.y_ed_[_new_ed.y_ed_[0] >_new_ed.y_ed_[1]] *= -1;
      THROW_IF((fabs(_new_ed.y_ed_[1] - _new_ed.y_ed_[0]) - dist_3d) > 1e-8,
               "Wrong y reange");
    };
    compute_x_y_ed(v2, new_edd);

    Geo::Interval<double> tmp;
    for (int i = 0; i < 2; ++i)
    {
      bool done = false;
      for (int j = 0; j < 2; ++j)
      {
        double t, dist_sq;
        Gen::closest_point<double, 2, true>(
          parent_trace_segs[i], v2[j], nullptr, &t, &dist_sq);
        done = dist_sq < Geo::epsilon_sq(v2[j]);
        if (done)
        {
          tmp.add(j);
          break;
        }
      }
      if (done)
        continue;

      double pars[2], dist_sq;
      Gen::closest_point<double, 2, true, true>(
        parent_trace_segs[i], v2, nullptr, pars, &dist_sq);
      auto dir = v2[1] - v2[0];
      double cr_pr = fabs(parent_trace_segs[i][1] % dir);
      if (cr_pr < 1e-12)
        pars[1] = Geo::length_square(v2[1]) > Geo::length_square(v2[0]) ? 1: 0;
      //auto val_sq = std::max(Geo::length_square(v2[0]), Geo::length_square(v2[1]));
      else if (pars[0] <= 1e-12)
      {
        if (pars[1] <= 0) pars[1] = 1;
        else if (pars[1] >= 1) pars[1] = 0;
        else 
          THROW("Bad par");
        
      }
      else if (pars[0] <= 1 - 1e-12)
        continue;
      tmp.add(std::clamp(pars[1], 0., 1.));
    }

    if (!tmp.empty())
    {
      new_edd.param_range_ = tmp;
      new_edd.dist0_ = _parent->second.dist0_;
      new_edd.init(_f, _parent);
      merge(fe, new_edd);
    }

    size_t par_bndr_to_fill = idx;
    if (tmp.empty() &&
        (_parent->second.param_range_[par_bndr_to_fill] == par_bndr_to_fill))
    {
      EdgeDistance new_edd2;
      new_edd2.dist0_ = _parent->second.get_distance(static_cast<double>(par_bndr_to_fill));
      auto orig = Geo::VectorD2{ _parent->second.x_, _parent->second.y_ed_[par_bndr_to_fill] };
      Gen::Segment<double, 2> new_seg;
      new_seg[0] = v2[0] - orig;
      new_seg[1] = v2[1] - orig;
      compute_x_y_ed(new_seg, new_edd2);
      new_edd2.param_range_ = { 0., 1. };
      new_edd2.init(_f, _parent);
      merge(fe, new_edd2);
    }
    par_bndr_to_fill = !idx;
    size_t new_par = !inv;
    if (!tmp.empty() && tmp[new_par] != new_par &&
      (_parent->second.param_range_[par_bndr_to_fill] == par_bndr_to_fill))
    {
      EdgeDistance new_edd2;
      new_edd2.dist0_ = _parent->second.get_distance(static_cast<double>(par_bndr_to_fill));
      auto orig = Geo::VectorD2{ _parent->second.x_, _parent->second.y_ed_[par_bndr_to_fill] };
      Gen::Segment<double, 2> new_seg;
      new_seg[0] = v2[0] - orig;
      new_seg[1] = v2[1] - orig;
      compute_x_y_ed(new_seg, new_edd2);
      if (inv)
        new_edd2.param_range_ = Geo::Interval<double>(0., tmp[0]);
      else
        new_edd2.param_range_ = Geo::Interval<double>(tmp[1], 1.);
      new_edd2.init(_f, _parent);
      merge(fe, new_edd2);
    }

  }
  THROW_IF(!oth_eds[0].ed_.get() || !oth_eds[1].ed_.get(), "");
}

static std::bitset<2> intersect(EdgeDistance& _a, EdgeDistance& _b,
                                Geo::Interval<double> _extras[2])
{
  auto inters_par_range = _a.param_range_ * _b.param_range_;
  if (inters_par_range.length() < 1e-12)
    return 0;
  double root;
  auto a_minus_b = find_minimum(_a, _b, root, inters_par_range);
  std::bitset<2> res;
  if (a_minus_b > 0)
    res[0] = _a.param_range_.subtract(inters_par_range, _extras[0]);
  else if (a_minus_b < 0)
    res[1] = _b.param_range_.subtract(inters_par_range, _extras[1]);
  else
  {
    auto snap_to_boundary = [](double& _root, const Geo::Interval<double>& _interv)
    {
      for (int i = 0; i < 2; ++i)
        if (fabs(_root - _interv[i]) < 1e-8)
        {
          _root = _interv[i];
          return true;
        }
      return false;
    };
    snap_to_boundary(root, _a.param_range_) || snap_to_boundary(root, _b.param_range_);
    Geo::Interval<double> near_a(Geo::Interval<double>::min(), root);
    Geo::Interval<double> near_b(root, Geo::Interval<double>::max());
    if (_a.get_distance(root - 1) > _b.get_distance(root - 1))
      std::swap(near_a, near_b);

    near_b *= _b.param_range_;
    res[0] = _a.param_range_.subtract(near_b, _extras[0]);
    near_a *= _a.param_range_;
    res[1] = _b.param_range_.subtract(near_a, _extras[1]);
  }
  return res;
}

void GeodesicDistance::merge(Topo::Wrap<Topo::Type::EDGE> _ed,
                             EdgeDistance& _ed_dist)
{
  auto range = edge_distances_.equal_range(_ed);
  Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> edv(_ed);
  if (edv.get(0)->id() == 395 && edv.get(1)->id() == 436)
  {
    std::cout << "Here we are";
  }
  std::vector<EdgeDistance*> old_ed_dists;
  for (auto it = range.first; it != range.second; ++it)
    old_ed_dists.push_back(&(it->second));
  std::vector<EdgeDistance> new_edds;
  new_edds.push_back(_ed_dist);
  for (auto old_edd : old_ed_dists)
  {
    //if (it->second.status_ != EdgeDistance::Status::Keep)
    if (old_edd->status_ == EdgeDistance::Status::Remove)
      continue;
    auto size = new_edds.size();
    for (size_t i = 0; i < size; ++i)
    {
      auto& new_edd = new_edds[i];
      Geo::Interval<double> extras[2];
      auto int_res = intersect(new_edd, *old_edd, extras);
      if (!int_res.any())
        continue;
      if (int_res[1])
      {
        if (old_edd->param_range_.empty())
        {
          std::function<void(EdgeDistance* _ed_dist)> skip_children =
            [&](EdgeDistance* _ed_dist)
          {
            _ed_dist->status_ = EdgeDistance::Status::Remove;
            for (auto child : _ed_dist->children_)
              if (child != nullptr)
                skip_children(&child->second);
          };
          skip_children(old_edd);
        }
        else
        {
          old_edd->update_parameter();
          if (!extras[1].empty())
          {
            auto split_piece = *old_edd;
            split_piece.param_range_ = extras[1];
            split_piece.update_parameter();
            insert_element(_ed, split_piece);
          }
        }
      }
      if (int_res[0])
      {
        new_edd.update_parameter();
        if (!extras[0].empty())
        {
          auto new_el = new_edds.emplace_back(new_edd);
          new_el.param_range_ = extras[0];
          new_el.update_parameter();
        }
      }
    }
  }
  for (auto& new_edd : new_edds)
    if (!new_edd.param_range_.empty())
      insert_element(_ed, new_edd);
}

bool GeodesicDistance::find_graph(
  double _dist,
  std::vector<Geo::VectorD3>& _pts,
  std::vector<std::array<size_t, 2>>& _inds)
{
  using FacePointMap = std::map<Topo::Wrap<Topo::Type::FACE>, std::vector<size_t>>;
  FacePointMap fp_map;
  for (auto& ed_dist : edge_distances_)
  {
    std::array<double, 2> pars;
    auto sol_num = ed_dist.second.parameters(_dist, pars);
    if (sol_num == 0)
      continue;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> vert_it(ed_dist.first);
    Geo::Segment seg;
    vert_it.get(0)->geom(seg[0]);
    vert_it.get(1)->geom(seg[1]);
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> face_it(ed_dist.first);
    for (int i = 0; i < sol_num; ++i)
    {
      auto pt = Geo::evaluate(seg, pars[i]);
      for (auto f : face_it)
        fp_map[f].push_back(_pts.size());
      _pts.push_back(pt);
    }
  }
  for (auto& fp : fp_map)
  {
    for (auto it = fp.second.begin(); it != fp.second.end(); ++it)
      for (auto it1 = it; ++it1 != fp.second.end(); )
        _inds.push_back({ *it, *it1 });
  }
  return true;
}

} // namespace Offset

#endif