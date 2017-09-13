#include "split_chain.hh"
#include "Geo/plane_fitting.hh"
#include "Utils/statistics.hh"

#include <array>
#include <set>

namespace Topo {

struct SplitChain : public ISplitChain
{
  virtual void add_chain(const VertexChain _chain)
  {
    result_.push_back(_chain);
  }
  virtual void add_connection(const Topo::Wrap<Topo::Type::VERTEX>& _v0,
                              const Topo::Wrap<Topo::Type::VERTEX>& _v1,
                              bool _bidirectional = true)
  {
    connections_.emplace(Connection({ _v0, _v1 }));
    if (_bidirectional)
      connections_.emplace(Connection({ _v1, _v0 }));
  }
  const VertexChains& split();

private:
  typedef std::array<Topo::Wrap<Topo::Type::VERTEX>, 2> Connection;
  typedef std::set<Connection> Connections;

  VertexChain find_chain(Connections::iterator _conns_it);
  double find_angle(const Connection& _a, const Connection& _b);

  VertexChain follow_chain(const Connection& _conn, 
                           std::set<Topo::Wrap<Topo::Type::VERTEX>>& _all_vert_ch);

  VertexChains result_;
  Connections connections_;
  Geo::Vector3 norm_;
};

std::shared_ptr<ISplitChain> ISplitChain::make()
{
  return std::make_shared<SplitChain>();
}

const VertexChains& SplitChain::split()
{
  if (result_.empty())
    return result_;
  norm_ = Geo::vertex_polygon_normal(result_[0].begin(), result_[0].end());
  for (auto conn_it = connections_.begin(); conn_it != connections_.end(); )
  {
    auto new_ch = find_chain(conn_it);
    ++conn_it;
    if (new_ch.empty())
      continue;
    auto& prev_vert = new_ch.back();
    for (auto& vert : new_ch)
    {
      auto used_conn_it = connections_.find(Connection{ prev_vert , vert });
      if (used_conn_it != connections_.end())
      {
        if (used_conn_it == conn_it)
          ++conn_it;
        connections_.erase(used_conn_it);
      }
    }
    result_.push_back(std::move(new_ch));
  }
  std::set<Topo::Wrap<Topo::Type::VERTEX>> all_chain_vertices;
  for (auto& ch : result_)
    for (auto& v : ch)
      all_chain_vertices.insert(v);

  for (auto conn_it = connections_.begin(); conn_it != connections_.end(); )
  {
    if (all_chain_vertices.find((*conn_it)[0]) == all_chain_vertices.end())
      continue;
    VertexChain new_ch = follow_chain(*conn_it, all_chain_vertices);
    if (new_ch.empty())
      continue;
  }

  return result_;
}

VertexChain SplitChain::find_chain(Connections::iterator _conns_it)
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
      auto norm = Geo::vertex_polygon_normal(result.begin(), result.end());
      if (norm_ * norm > 0)
        break;
      result.clear();
    }

    auto& end_v = edge[1];
    auto pos =
      connections_.lower_bound(Connection{ end_v, Topo::Wrap<Topo::Type::VERTEX>() });
    double min_ang = std::numeric_limits<double>::max();
    std::vector<std::tuple<double, Connections::iterator>> choices;
    const double ANG_EPS = 1e-8;
    for (; pos != connections_.end() && (*pos)[0] == end_v; )
    {
      auto ang = find_angle(edge, *pos);
      Utils::a_eq_b_if_a_gt_b(min_ang, ang);
      if (ang > 2 * M_PI - ANG_EPS && (*pos)[1] != edge[0])
        ang = 0;
      choices.emplace_back(ang, pos);
    }
    for (const auto& achoice : choices)
    {
      if (std::get<double>(achoice) > min_ang + ANG_EPS)
        continue;
      branches.emplace_back(std::get<Connections::iterator>(achoice), branches.size());
    }
  }
  return result;
}

double SplitChain::find_angle(const Connection& _a, const Connection& _b)
{
  auto make_vector = [](const Connection& _con)
  {
    Geo::Point pts[2];
    _con[0]->geom(pts[0]);
    _con[1]->geom(pts[1]);
    return pts[1] - pts[0];
  };
  auto v0 = make_vector(_a);
  auto v1 = make_vector(_b);
  auto ang = Geo::signed_angle(-v0, v1, norm_);
  if (ang < 0)
    ang = 2 * M_PI + ang;
  return ang;
}

VertexChain SplitChain::follow_chain(
  const Connection& _conn,
  std::set<Topo::Wrap<Topo::Type::VERTEX>>& _all_vert_ch)
{
  Connection curr_conn = _conn;
  VertexChain v_ch;
  v_ch.push_back(curr_conn[0]);
  v_ch.push_back(curr_conn[1]);
  while (_all_vert_ch.find(v_ch.back()) == _all_vert_ch.end())
  {
    auto pos =
      connections_.lower_bound(Connection{ curr_conn[1], Topo::Wrap<Topo::Type::VERTEX>() });
    if (pos == connections_.end())
      return VertexChain();
    curr_conn = *pos;
    v_ch.push_back(curr_conn[1]);
  }
  return v_ch;
}


} // namespace Topo