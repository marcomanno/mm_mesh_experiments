//#pragma optimize ("", off)
#include "split_chain.hh"
#include "geom.hh"

#include "Geo/plane_fitting.hh"
#include "PolygonTriangularization/poly_triang.hh"
#include "Utils/error_handling.hh"
#include "Utils/circular.hh"
#include "Utils/statistics.hh"

#include <array>
#include <functional>
#include <map>
#include <set>

namespace Topo {

namespace
{
using Connection = std::array<Topo::Wrap<Topo::Type::VERTEX>, 2>;
using Connections = std::set<Connection>;
} // namespace

struct SplitChain : public ISplitChain
{
  virtual void add_chain(const VertexChain _chain)
  {
    boundaries_.push_back(_chain);
  }
  virtual void add_connection(const Topo::Wrap<Topo::Type::VERTEX>& _v0,
                              const Topo::Wrap<Topo::Type::VERTEX>& _v1,
                              bool _bidirectional = true) override
  {
    connections_.emplace(Connection({ _v0, _v1 }));
    if (_bidirectional)
      connections_.emplace(Connection({ _v1, _v0 }));
  }
  ConnectionCheck check_new_connection(
    const Topo::Wrap<Topo::Type::VERTEX>& _v0,
    const Topo::Wrap<Topo::Type::VERTEX>& _v1) const override;

  void compute();
  const VertexChains& boundaries() const override { return boundaries_; }
  const VertexChains* boundary_islands(size_t _bondary_ind) const override
  {
    auto isl_it = islands_.find(_bondary_ind);
    if (isl_it == islands_.end())
      return nullptr;
    else
      return &isl_it->second;
  }

private:
  VertexChain find_chain(Connections::iterator _conns_it);
  double find_angle(const Topo::Wrap<Topo::Type::VERTEX>& _a,
                    const Topo::Wrap<Topo::Type::VERTEX>& _b,
                    const Topo::Wrap<Topo::Type::VERTEX>& _c);

  size_t find_boundary_index(const VertexChain& _ch) const;

  void remove_chain_from_connection(
    VertexChain& _ch,
    Connections::iterator* _conn_it,
    bool _open);

  VertexChain follow_chain(const Connection& _conn,
                           std::set<Topo::Wrap<Topo::Type::VERTEX>>& _all_vert_ch);
  bool locate(const VertexChain& _ch, 
              size_t _loop_inds[2],
              std::array<size_t, 2>& _pos);

  void split_boundary_chain(
    size_t _chain_ind, std::array<size_t, 2>& _ins,
    const VertexChain& _new_ch);

  void merge_boundary_chains(
    size_t _chain_inds[2], std::array<size_t, 2>& _ins,
    const VertexChain& _new_ch);

  VertexChains boundaries_;
  std::map<size_t, VertexChains> islands_;
  Connections connections_;
  Geo::VectorD3 norm_;
};

std::shared_ptr<ISplitChain> ISplitChain::make()
{
  return std::make_shared<SplitChain>();
}

ISplitChain::ConnectionCheck SplitChain::check_new_connection(
  const Topo::Wrap<Topo::Type::VERTEX>& _v0,
  const Topo::Wrap<Topo::Type::VERTEX>& _v1) const
{
  Geo::Segment seg_new;
  _v0->geom(seg_new[0]);
  _v1->geom(seg_new[1]);
  Geo::Segment seg;
  for (const auto& conn : connections_)
  {
    if (_v0 == conn[0] || _v1 == conn[0] || 
        _v0 == conn[1] || _v1 == conn[1])
      continue;
    conn[0]->geom(seg[0]);
    conn[1]->geom(seg[1]);
    if (Geo::closest_point(seg, seg_new))
      return ConnectionCheck::INVALID;
  }
  for (const auto& chain : boundaries_)
  {
    auto vp = chain.back();
    for (const auto& v : chain)
    {
      if (_v0 != vp && _v1 != vp && _v0 != v && _v1 != v)
      {
        vp->geom(seg[0]);
        v->geom(seg[1]);
        if (Geo::closest_point(seg, seg_new))
          return ConnectionCheck::INVALID;
      }
      vp = v;
    }
  }   
  return ConnectionCheck::OK;
}

void SplitChain::compute()
{
  if (boundaries_.empty())
    return;
  norm_ = Geo::vertex_polygon_normal(boundaries_[0].begin(), boundaries_[0].end());
  std::set<Topo::Wrap<Topo::Type::VERTEX>> all_chain_vertices;
  for (auto& ch : boundaries_)
    for (auto& v : ch)
      all_chain_vertices.insert(v);
  for (bool achange = true; achange;)
  {
    achange = false;
    for (auto conn_it = connections_.begin(); conn_it != connections_.end(); )
    {
      auto cur_conn_it = conn_it++;
      if (all_chain_vertices.find((*cur_conn_it)[0]) == all_chain_vertices.end())
        continue;
      VertexChain new_ch = follow_chain(*cur_conn_it, all_chain_vertices);
      if (new_ch.empty())
        continue;
      achange = true;
      std::array<size_t, 2> ins;
      size_t chain_inds[2];
      THROW_IF(!locate(new_ch, chain_inds, ins), "No attah chain");
      if (chain_inds[0] == chain_inds[1])
        split_boundary_chain(chain_inds[0], ins, new_ch);
      else
        merge_boundary_chains(chain_inds, ins, new_ch);
      remove_chain_from_connection(new_ch, &conn_it, true);
      if (chain_inds[0] != chain_inds[1] || ins[0] != ins[1])
      {
        std::reverse(new_ch.begin(), new_ch.end());
        remove_chain_from_connection(new_ch, &conn_it, true);
      }

      all_chain_vertices.insert(new_ch.begin(), new_ch.end());
    }
  }

  VertexChains islands;
  for (auto conn_it = connections_.begin(); 
       conn_it != connections_.end(); )
  {
    auto new_ch = find_chain(conn_it);
    ++conn_it;
    if (new_ch.empty())
      continue;

    auto norm = Geo::vertex_polygon_normal(new_ch.begin(), new_ch.end());
    VertexChains* loops[2] = { &boundaries_, &islands };
    if (norm_ * norm < 0)
      std::swap(loops[0], loops[1]);
    loops[0]->push_back(new_ch);

    remove_chain_from_connection(new_ch, &conn_it, false);
  }
  for (auto& ch : islands)
  {
    auto ind = find_boundary_index(ch);
    islands_[ind].push_back(std::move(ch));
  }
}

VertexChain SplitChain::find_chain(
  Connections::iterator _conns_it)
{
  std::vector<Connections::iterator> edge_ch;
  std::vector<std::tuple<Connections::iterator, size_t>> branches;
  branches.emplace_back(_conns_it, 0);
  VertexChain result;
  while (!branches.empty())
  {
    edge_ch.resize(std::get<size_t>(branches.back()) + 1);
    edge_ch.back() = std::get<Connections::iterator>(branches.back());
    branches.pop_back();
    const auto& edge = *edge_ch.back();
    if (edge[1] == (*(edge_ch[0]))[0])
    {
      for (auto& ed : edge_ch)
        result.push_back((*ed)[0]);
      break;
    }

    auto& end_v = edge[1];
    auto pos =
      connections_.lower_bound(Connection{ end_v, Topo::Wrap<Topo::Type::VERTEX>() });
    double min_ang = std::numeric_limits<double>::max();
    std::vector<std::tuple<double, Connections::iterator>> choices;
    const double ANG_EPS = 1e-8;
    for (; pos != connections_.end() && (*pos)[0] == end_v; ++pos)
    {
      auto ang = find_angle(edge[0], edge[1], (*pos)[1]);
      Utils::a_eq_b_if_a_gt_b(min_ang, ang);
      if (ang > 2 * M_PI - ANG_EPS && (*pos)[1] != edge[0])
        ang = 0;
      choices.emplace_back(ang, pos);
    }
    for (const auto& achoice : choices)
    {
      if (std::get<double>(achoice) > min_ang + ANG_EPS)
        continue;
      branches.emplace_back(std::get<Connections::iterator>(achoice), edge_ch.size());
    }
  }
  return result;
}

double SplitChain::find_angle(
  const Topo::Wrap<Topo::Type::VERTEX>& _a, 
  const Topo::Wrap<Topo::Type::VERTEX>& _b,
  const Topo::Wrap<Topo::Type::VERTEX>& _c)
{
  if (_a == _c)
    return 2 * M_PI;
  Geo::Point pts[3];
  _a->geom(pts[0]);
  _b->geom(pts[1]);
  _c->geom(pts[2]);
  auto v0 = pts[0] - pts[1];
  auto v1 = pts[2] - pts[1];
  auto ang = Geo::signed_angle(v1, v0, norm_);
  if (ang < 0)
    ang += 2 * M_PI;
  return ang;
}

VertexChain SplitChain::follow_chain(
  const Connection& _conn,
  std::set<Topo::Wrap<Topo::Type::VERTEX>>& _all_vert_ch)
{
  VertexChain v_ch;
  v_ch.push_back(_conn[0]);
  v_ch.push_back(_conn[1]);
  std::set<const Connection*> used_conn;
  while (_all_vert_ch.find(v_ch.back()) == _all_vert_ch.end())
  {
    auto pos =
      connections_.lower_bound(Connection{ v_ch.back(), Topo::Wrap<Topo::Type::VERTEX>() });
    for (;; ++pos)
    {
      if (pos == connections_.end() || (*pos)[0] != v_ch.back())
        return VertexChain();
      if (used_conn.find(&(*pos)) == used_conn.end())
        break;
    }
    const Connection& curr_conn = *pos;
    used_conn.insert(&curr_conn);
    v_ch.push_back(curr_conn[1]);
  }
  return v_ch;
}

bool SplitChain::locate(
  const VertexChain& _ch,
  size_t _loop_inds[2],
  std::array<size_t, 2>& _pos)
{
  typedef std::array<size_t, 2> InsPoint;
  std::vector<InsPoint> choices[2];
  for (auto i = boundaries_.size(); i-- > 0;)
  {
    for (auto j = boundaries_[i].size(); j-- > 0;)
    {
      if (boundaries_[i][j] == _ch.front())
        choices[0].push_back({ i, j });
      if (boundaries_[i][j] == _ch.back())
        choices[1].push_back({ i, j });
    }
  }
  // Start and end points must be inside the same chain.
  // Remove ins points that are using chains not present in
  // the other point.
  for (size_t i = 0; i < std::size(choices); ++i)
  {
    if (choices[i].size() < 2)
      continue;
    for (auto j = choices[i].size(); j-- > 0;)
    {
      auto it_ch =
        std::lower_bound(choices[1 - i].begin(), choices[1 - i].end(),
                         InsPoint({ choices[i][j][0], std::numeric_limits<size_t>::max() }),
                         std::greater<InsPoint>());
      if (it_ch == choices[1 - i].end() || choices[i][j][0] != (*it_ch)[0])
      {
        THROW_IF(choices[i].size() <= 1, "No choices left");
        choices[i].erase(choices[i].begin() + j);
      }
    }
  }
  // Select the chain
  if (choices[0].front()[0] != choices[0].back()[0])
  {
    Geo::Point pt_in;
    _ch[1]->geom(pt_in);
    if (_ch.size() == 2)
    {
      Geo::Point pt_0;
      _ch[0]->geom(pt_0);
      pt_in += pt_0;
      pt_in /= 2.;
    }
    // There is more than one chain.
    size_t prev_chain_ind = std::numeric_limits<size_t>::max();
    size_t sel_chain_ind = std::numeric_limits<size_t>::max();
    for (auto& ins_pt : choices[0])
    {
      if (prev_chain_ind == ins_pt[0])
        continue;

      prev_chain_ind = ins_pt[0];
      auto pt_cl = PointInFace::classify(boundaries_[ins_pt[0]], pt_in);
      if (pt_cl == Geo::PointInPolygon::Classification::Inside)
      {
        sel_chain_ind = ins_pt[0];
        break;
      }
    }
    THROW_IF(sel_chain_ind == std::numeric_limits<size_t>::max(),
             "no loop contains the chain");
    for (auto& ch : choices)
      for (auto i = ch.size(); i-- > 0;)
        if (ch[i][0] != sel_chain_ind)
          ch.erase(ch.begin() + i);
  }
  // Select the insertion point.
  Topo::Wrap<Topo::Type::VERTEX> vv[2] = { _ch[1], _ch[_ch.size() - 2] };
  for (auto i : { 0, 1 })
  {
    auto& cur = choices[i];
    if (cur.size() < 2)
      continue;
    for (auto j = cur.size(); j-- > 0; )
    {
      auto& achoice = cur[j];
      auto& sel_ch = boundaries_[achoice[0]];
      Topo::Wrap<Topo::Type::VERTEX>* vert_ptrs[4] = {
        &sel_ch[Utils::decrease(achoice[1], sel_ch.size())],
        &sel_ch[achoice[1]],
        &sel_ch[Utils::increase(achoice[1], sel_ch.size())],
        &vv[i]
      };
      auto out_angle = find_angle(*vert_ptrs[0], *vert_ptrs[1], *vert_ptrs[2]);
      auto int_angle = find_angle(*vert_ptrs[0], *vert_ptrs[1], vv[i]);
      if (int_angle > out_angle)
        cur.erase(cur.begin() + j);
    }
  }
  if (choices[0].size() != 1 || choices[1].size() != 1)
    return false;
  for (auto i : { 0, 1 })
  {
    _loop_inds[i] = choices[i].front()[0];
    _pos[i] = choices[i].front()[1];
  }
  return true;
}

void SplitChain::split_boundary_chain(
  size_t _chain_ind, std::array<size_t, 2>& _ins,
  const VertexChain& _new_ch)
{
  VertexChain split_chains[2];
  bool curr_chain = _ins[0] > _ins[1];
  for (size_t i = 0; i < boundaries_[_chain_ind].size(); ++i)
  {
    if (i != _ins[0] && i != _ins[1])
      split_chains[curr_chain].push_back(boundaries_[_chain_ind][i]);
    else
    {
      if (i == _ins[0])
        split_chains[0].insert(split_chains[0].end(), _new_ch.begin(), _new_ch.end());
      else
        split_chains[1].insert(split_chains[1].end(), _new_ch.rbegin(), _new_ch.rend());
      if (_ins[0] != _ins[1])
        curr_chain = !curr_chain;
    }
  }
  boundaries_[_chain_ind] = std::move(split_chains[0]);
  if (_ins[0] != _ins[1])
    boundaries_.push_back(std::move(split_chains[1]));
}

void SplitChain::merge_boundary_chains(
  size_t _chain_inds[2], std::array<size_t, 2>& _ins,
  const VertexChain& _new_ch)
{
  VertexChain merged_loop;
  for (size_t i = 0; i < boundaries_[_chain_inds[0]].size(); ++i)
  {
    if (i != _ins[0])
      merged_loop.push_back(boundaries_[_chain_inds[0]][i]);
    else
    {
      merged_loop.insert(merged_loop.end(), _new_ch.begin(), _new_ch.end());
      auto& bndr2 = boundaries_[_chain_inds[1]];
      for (size_t j = Utils::increase(_ins[1], bndr2.size());
           j != _ins[1]; j = Utils::increase(j, bndr2.size()))
      {
        merged_loop.push_back(bndr2[j]);
      }
      merged_loop.insert(merged_loop.end(), _new_ch.rbegin(), _new_ch.rend());
    }
  }
  std::sort(_chain_inds, _chain_inds + 2);
  boundaries_[_chain_inds[0]] = std::move(merged_loop);
  boundaries_.erase(boundaries_.begin() + _chain_inds[1]);
}

void SplitChain::remove_chain_from_connection(
  VertexChain& _ch,
  Connections::iterator* _conn_it,
  bool _open)
{
  auto prev_vert_it = _open ? _ch.begin() : std::prev(_ch.end());
  for (auto vert_it = _ch.begin() + _open;  vert_it != _ch.end();
       prev_vert_it = vert_it++)
  {
    auto remove_connection = [this, _conn_it](
      const Wrap<Type::VERTEX>& _a,
      const Wrap<Type::VERTEX>& _b)
    {
      auto used_conn_it = connections_.find(Connection{ _a , _b });
      if (used_conn_it != connections_.end())
      {
        if (_conn_it != nullptr && used_conn_it == *_conn_it)
          ++(*_conn_it);
        connections_.erase(used_conn_it);
      }
    };
    remove_connection(*prev_vert_it, *vert_it);
  }
}

size_t SplitChain::find_boundary_index(const VertexChain& _ch) const
{
  THROW_IF(_ch.empty(), "Empty chain ov vertices");
  Geo::Point pt;
  _ch[0]->geom(pt);
  std::vector<size_t> choices;
  for (auto i = boundaries_.size(); i-- > 0; )
  {
    if (PointInFace::classify(boundaries_[i], pt) ==
        Geo::PointInPolygon::Classification::Inside)
    {
      choices.push_back(i);
    }
  }
  THROW_IF(choices.empty(), "Island without a boundary");
  if (choices.size() == 1)
    return choices[0];
  double min_area = std::numeric_limits<double>::max();
  size_t min_ind = 0;
  for (auto i : choices)
  {
    std::vector<Geo::Point> pts;
    pts.reserve(boundaries_[i].size());
    for (auto& v : boundaries_[i])
    {
      pts.emplace_back();
      v->geom(pts.back());
    }
    auto poly_t = Geo::IPolygonTriangulation::make();
    poly_t->add(pts);
    auto area = poly_t->area();
    if (Utils::a_eq_b_if_a_gt_b(min_area, area))
      min_ind = i;
  }
  return min_ind;
}

} // namespace Topo
