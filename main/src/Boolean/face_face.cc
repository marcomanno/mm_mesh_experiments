#include "face_intersections.hh"
#include "Base/basic_type.hh"
#include "Geo/kdtree.hh"
#include "Geo/plane_fitting.hh"
#include "Geo/vector.hh"
#include <Geo/point_in_polygon.hh>
#include "Topology/connect.hh"
#include "Topology/geom.hh"
#include <Topology/impl.hh>
#include <Topology/shared.hh>
#include <Topology/split.hh>
#include "Utils/error_handling.hh"
#include "Utils/circular.hh"
#include "Utils/graph.hh"

#include <list>
#include <set>

namespace Boolean {

namespace {

std::set<Topo::Wrap<Topo::Type::VERTEX>> face_vertices(
  const std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _verts,
  const Topo::Wrap<Topo::Type::FACE>& _face_a)
{
  std::set<Topo::Wrap<Topo::Type::VERTEX>> vert_set;
  vert_set.insert(_verts.begin(), _verts.end());
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(_face_a);
  vert_set.insert(fv_it.begin(), fv_it.end());
  return vert_set;
}

bool boundary_chain(
  const Topo::Wrap<Topo::Type::FACE>& _face,
  const std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _v_inters,
  Topo::Wrap<Topo::Type::VERTEX> _start_end[2])
{
  _start_end[0].reset(nullptr);
  _start_end[1].reset(nullptr);
  Topo::Iterator<Topo::Type::FACE, Topo::Type::COEDGE> coe_it(_face);
  std::map<Topo::Wrap<Topo::Type::VERTEX>, Base::BasicType<int>> verts_use;
  for (auto coe : coe_it)
  {
    Topo::Iterator<Topo::Type::COEDGE, Topo::Type::VERTEX> vert_it(coe);
    bool good = true;
    THROW_IF(vert_it.size() != 2, "Coedgewithout 2 vertices");
    for (auto vert : vert_it)
      good &= std::find(_v_inters.begin(), _v_inters.end(), vert) != _v_inters.end();
    if (!good)
      continue;
    for (size_t i = 0; i < 2; ++i)
      verts_use[vert_it.get(i)] += 1 << i;
  }
  if (verts_use.size() < _v_inters.size())
    return false;
  for (auto& vert_use : verts_use)
  {
    switch (vert_use.second)
    {
    case 1:
    case 2:
      if (_start_end[vert_use.second - 1])
        return false;
      _start_end[vert_use.second - 1] = vert_use.first;
    case 3:
      break;
    default:
      return false;
    }
  }
  if (!_start_end[0] || !_start_end[1])
    return false;
  else
    return true;
}

struct FaceEdgeMap
{
  typedef std::vector<Topo::Wrap<Topo::Type::VERTEX>> CommonVerticesBasic;
  struct CommonVertices : public CommonVerticesBasic
  {
    CommonVertices(const CommonVerticesBasic& _comm_vert, bool _doubious) :
      CommonVerticesBasic(_comm_vert), doubious_(_doubious) {}
    bool doubious_ = false;
  };
  typedef std::vector<CommonVertices> NewVerts;
  typedef std::vector<Topo::Wrap<Topo::Type::FACE>> NewFaces;

  bool add_face_edge(
    const Topo::Wrap<Topo::Type::FACE>& _face,
    const std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _v_inters,
    bool _is_face_b, bool _doubious)
  {
    auto& map = map_[_is_face_b];
    auto elem = map.emplace(_face, FaceData());

    auto& data = elem.first->second;
    data.new_verts_.emplace_back(_v_inters, _doubious);

    if (elem.second) // It is a new face
    {
      Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(_face);
      auto pl_fit = Geo::IPlaneFit::make();
      pl_fit->init(fv_it.size());
      std::vector<Geo::Point> face_pts;
      for (auto vert : fv_it)
      {
        face_pts.emplace_back();
        vert->geom(face_pts.back());
        pl_fit->add_point(face_pts.back());
      }
      Geo::Point c, n;
      pl_fit->compute(c, n);
      auto pl_geom = Geo::IPolygonalFace::make();
      pl_geom->add_loop(face_pts.begin(), face_pts.end());
      pl_geom->compute();
      if (pl_geom->normal() * n < 0)
        n = -n;
      data.norm_ = n;
    }
    return true;
  }

  void init_map();
  void split(OverlapFces& _overlap_faces);

private:
  typedef Geo::Point Normal;
  struct FaceData
  {
    Normal norm_;
    NewFaces new_faces_;
    NewVerts new_verts_;
  };

  std::map<Topo::Wrap<Topo::Type::FACE>, FaceData> map_[2];
  void check_doubious_faces();
  void split_overlaps(OverlapFces&  _overlap_faces);
  bool split_on_boundary(
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX>& it_ev,
    bool first_is_common, bool prev_is_common,
    Topo::VertexChains& comm_verts, Topo::VertexChains& not_comm_verts,
    CommonVertices& edge_set_copy,
    OverlapFcesVectror& _overlap_faces,
    Topo::Wrap<Topo::Type::FACE>& face,
    Geo::Vector3& face_normal,
    Topo::VertexChains& split_chains);

    void split_with_chains();
};

typedef std::vector<FaceEdgeMap::NewVerts::iterator> EdgeChain;

// Finds a set of connected edges (couple of vertices) and using the Edges in _pt_sets
bool find_edge_chain(
  const Topo::Wrap<Topo::Type::VERTEX>& _vert,
  FaceEdgeMap::NewVerts& _ed_sets,
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX>& _fv_it,
  EdgeChain& _edge_ch,
  Topo::Wrap<Topo::Type::VERTEX>*& _last_vert)
{
  auto follow_chain = [&_ed_sets, &_fv_it, &_last_vert, &_vert](
    FaceEdgeMap::NewVerts::iterator _edge_itr,
    EdgeChain& _edge_ch)
  {
    std::vector<bool> used(_ed_sets.size(), false);
    used[_edge_itr - _ed_sets.begin()] = true;
    _edge_ch.push_back(_edge_itr);
    for (;;)
    {
      _last_vert = std::find(_fv_it.begin(), _fv_it.end(), (*_edge_itr)[1]);
      if (_last_vert != _fv_it.end())
        break;

      if (_vert == (*_edge_itr)[1])
      {
        _last_vert = nullptr;
        break;
      }

      bool found = false;
      for (auto pt_set_it = _ed_sets.begin(); pt_set_it != _ed_sets.end(); ++pt_set_it)
      {
        if (used[pt_set_it - _ed_sets.begin()] || pt_set_it->size() != 2)
          continue;
        auto& pt_set = *pt_set_it;
        if (pt_set[0] == (*_edge_itr)[1] || pt_set[1] == (*_edge_itr)[1])
        {
          if (pt_set[1] == (*_edge_itr)[1])
            std::swap(pt_set[1], pt_set[0]);
          _edge_itr = pt_set_it;
          used[_edge_itr - _ed_sets.begin()] = true;
          _edge_ch.push_back(_edge_itr);
          found = true;
          break;
        }
      }
      if (!found)
        return false;
    }
    // Check that the chain is inside the face.
    std::vector<Geo::Point> polygon;
    polygon.reserve(_fv_it.size());
    for (auto vert : _fv_it)
    {
      polygon.emplace_back();
      vert->geom(polygon.back());
    }
    Geo::Point pt;
    (*_edge_ch[0])[1]->geom(pt);
    if (_edge_ch.size() == 1)
    {
      Geo::Point pt0;
      (*_edge_ch[0])[0]->geom(pt0);
      pt = (pt + pt0) * 0.5;
    }
    auto pt_class = Geo::PointInPolygon::classify(polygon, pt);
    if (pt_class == Geo::PointInPolygon::Inside)
      return true;
    if (pt_class == Geo::PointInPolygon::On)
      THROW("Point not expected on boundary");
    _edge_ch.clear();
    return false;
  };

  auto edge_itr = _ed_sets.end();
  for (auto pt_it = _ed_sets.begin(); pt_it != _ed_sets.end(); ++pt_it)
  {
    if (pt_it->size() != 2)
      continue;
    auto iter = std::find(pt_it->begin(), pt_it->end(), _vert);
    if (iter != pt_it->end())
    {
      edge_itr = pt_it;
      if (iter != pt_it->begin())
        std::swap((*pt_it)[1], (*pt_it)[0]);

      _edge_ch.clear();
      if (follow_chain(edge_itr, _edge_ch))
        return true;
    }
  }
  return false;
}

std::shared_ptr<Geo::IPolygonalFace> make_polygonal_face(
  std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _verts)
{
  std::vector<Geo::Point> pts;
  pts.reserve(_verts.size());
  for (auto& vert : _verts)
  {
    pts.emplace_back();
    vert->geom(pts.back());
  }
  auto poly = Geo::IPolygonalFace::make();
  poly->add_loop(pts.begin(), pts.end());
  poly->compute();
  return poly;
}

Geo::PointInPolygon::Classification
point_in_polygon(
  std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _verts,
  const Geo::Point& _pt,
  const Geo::Vector3* _norm = nullptr)
{
  std::vector<Geo::Point> pts;
  pts.resize(_verts.size());
  for (auto& vert : _verts)
  {
    pts.emplace_back();
    vert->geom(pts.back());
  }
  return Geo::PointInPolygon::classify(pts, _pt, _norm);
}

} //namespace


bool FaceVersus::face_intersect(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_a,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_b)
{
  FaceEdgeMap face_new_edge_map;
  Geo::KdTree<Topo::Wrap<Topo::Type::FACE>> kdfaces_a;
  kdfaces_a.insert(_face_it_a.begin(), _face_it_a.end());
  kdfaces_a.compute();
  Geo::KdTree<Topo::Wrap<Topo::Type::FACE>> kdfaces_b;
  kdfaces_b.insert(_face_it_b.begin(), _face_it_b.end());
  kdfaces_b.compute();
  auto pairs = Geo::find_kdtree_couples<Topo::Wrap<Topo::Type::FACE>,
    Topo::Wrap<Topo::Type::FACE>>(kdfaces_a, kdfaces_b);
  for (const auto& pair : pairs)
  {
    const auto& face_a = kdfaces_a[pair[0]];
    const auto& face_b = kdfaces_b[pair[1]];
    const auto& vert_set_a = 
      face_vertices(f_vert_info_[face_a].new_vert_list_, face_a);
    const auto& vert_set_b =
      face_vertices(f_vert_info_[face_b].new_vert_list_, face_b);
    std::vector<Topo::Wrap<Topo::Type::VERTEX>> v_inters;
    std::set_intersection(
      vert_set_a.cbegin(), vert_set_a.cend(),
      vert_set_b.cbegin(), vert_set_b.cend(),
      std::back_inserter(v_inters));
    if (v_inters.size() < 2)
      continue;

    Topo::Wrap<Topo::Type::VERTEX> start_end_a[2], start_end_b[2];
    auto add_a = !boundary_chain(face_a, v_inters, start_end_a);
    auto add_b = !boundary_chain(face_b, v_inters, start_end_b);
    if (v_inters.size() > 2 && add_a != add_b)
      add_a = add_b = true;
    else if (v_inters.size() > 2 && !add_a && !add_b)
    {
      bool same_sense = false;
      if (start_end_a[0] == start_end_b[0] && start_end_a[1] == start_end_b[1])
        same_sense = true;
      else if (start_end_a[1] == start_end_b[0] && start_end_a[0] == start_end_b[1])
        same_sense = false;
      else
        add_a = add_b = true;
      if (!add_a && !add_b)
      {
        auto face_normal = [](const Topo::Wrap<Topo::Type::FACE>& _face)
        {
          Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> l_it(_face);
          Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> v_it(*l_it.begin());
          return Geo::vertex_polygon_normal(v_it.begin(), v_it.end());
        };
        auto cos_ang = face_normal(face_a) * face_normal(face_b);
        if (fabs(cos_ang) > 0.7)
        {
          bool same_dir = (face_normal(face_a) * face_normal(face_b)) > 0;
          add_a = same_dir == same_sense;
        }
        add_b = add_a;
      }
    }
    if (!add_a && !add_b)
      continue;
    // std::sort(v_inters.begin(), v_inters.end());
    if (add_a)
      face_new_edge_map.add_face_edge(face_a, v_inters, false, !add_b);
    if (add_b)
      face_new_edge_map.add_face_edge(face_b, v_inters, true, !add_a);
  }
  face_new_edge_map.init_map();
  face_new_edge_map.split(overlap_faces_);
  return true;
}

namespace {

void FaceEdgeMap::init_map()
{
  for (auto& map : map_)
    for (auto& face_info : map)
      face_info.second.new_faces_.push_back(face_info.first);
}

static bool insert_remaning_common_vertices(
  Topo::VertexChain& _verts,
  Topo::VertexChains& _connections,
  Topo::VertexChains& _split_chains)
{
  bool consumed;
  do
  {
    consumed = false;
    for (auto i = _verts.size(); i-- > 0;)
    {
      std::vector<std::array<size_t, 2>> conn_idxs;
      for (auto j = _connections.size(); j-- > 0;)
        for (auto k = _connections[j].size(); k-- > 0;)
        {
          auto edges = Topo::shared_entities
            <Topo::Type::VERTEX, Topo::Type::EDGE>
            (_verts[i], _connections[j][k]);
          if (edges.empty())
            continue;
          conn_idxs.push_back({ j, k });
        }
      if (conn_idxs.empty())
        continue;
      if (conn_idxs.size() > 1)
      {
        // Select the best chain to use for connection as the 
        // nearest to the overlap vertex to insert.
        Geo::Point ptv;
        _verts[i]->geom(ptv);
        double dist_sq_min = std::numeric_limits<double>::max();
        for (const auto& conn_idx : conn_idxs)
        {
          auto& chain = _connections[conn_idx[0]];
          Geo::Segment seg;
          chain.front()->geom(seg[0]);
          chain.back()->geom(seg[1]);
          double dist_sq;
          Geo::closest_point(seg, ptv, nullptr, nullptr, &dist_sq);
          if (dist_sq < dist_sq_min)
          {
            dist_sq_min = dist_sq;
            conn_idxs[0] = conn_idx;
          }
        }
        if (dist_sq_min == std::numeric_limits<double>::max())
          continue;
      }
      auto& chain = _connections[conn_idxs[0][0]];
      auto ind = conn_idxs[0][1];
      Topo::Wrap<Topo::Type::VERTEX> v0, v1;
      auto ind1 = ind;
      if (ind == 0) ++ind1;
      else if (ind == chain.size() - 1) --ind1;
      else
      {
        THROW("No good insertion point");
        continue;
      }
      v0 = chain[ind];
      v1 = chain[ind1];
      std::vector<std::array<size_t, 2>> insertions;
      for (size_t k = 0; k < _split_chains.size(); ++k)
      {
        auto& split_chain = _split_chains[k];
        auto prev = split_chain.front();
        for (auto j = split_chain.size(); j-- > 0; prev = split_chain[j])
        {
          if (prev == v0 && split_chain[j] == v1) {}
          else if (prev == v1 && split_chain[j] == v0) {}
          else
            continue;
          insertions.push_back({ k, j + 1 });
          break;
        }
      }
      if (insertions.size() >= 2)
      {
        if (insertions.size() > 2)
        {
          Geo::Point pt_v0;
          v0->geom(pt_v0);
          if (_split_chains[0].size() > 1)
            std::cout << "Unreliable test" << std::endl;
          for (auto it = insertions.end(); --it != insertions.begin(); )
          {
            auto& split_chain = _split_chains[(*it)[0]];
            if (point_in_polygon(split_chain, pt_v0) ==
                Geo::PointInPolygon::Classification::Outside)
            {
              it = insertions.erase(it);
            }
          }
          if (insertions.size() != 2)
            continue;
        }
        for (auto& ins : insertions)
        {
          auto& split_chain = _split_chains[ins[0]];
          split_chain.insert(split_chain.begin() + ins[1], _verts[i]);
        }
        chain[ind] = v0;
        _verts.erase(_verts.begin() + i);
        consumed = true;
      }
    }
  } while (consumed);
#if 0
  if (_verts.empty())
    return _verts.empty();

  Utils::Graph<Topo::E<Topo::Type::VERTEX>> graph;
  auto may_add_link = [&graph](
    Topo::Wrap<Topo::Type::VERTEX>& _v0, Topo::Wrap<Topo::Type::VERTEX>& _v1)
  {
    auto edges =
      Topo::shared_entities<Topo::Type::VERTEX, Topo::Type::EDGE>(_v0, _v1);
    if (!edges.empty())
      graph.add_link(_v1.get(), _v0.get());
  };
  for (auto i = _verts.size(); i-- > 0;)
  {
    for (auto& split_chain : _split_chains)
      for (auto& v : split_chain)
        may_add_link(_verts[i], v);
    for (auto j = i; j-- > 0;)
      may_add_link(_verts[i], _verts[j]);
  }
  graph.compute();
  for (size_t i = 0; i < graph.get_chain_number(); ++i)
  {
    for (auto& split_chain : _split_chains)
    {
      bool done = false;
      for (size_t k = 0; k < split_chain.size() && !done; ++k)
      {
        if (split_chain[k].get() != graph.get_chain_element(i, 0))
          continue;

        auto last = graph.get_chain_element_number(i) - 1;
        auto k1 = Utils::decrease(k, split_chain.size());
        auto ins = k;
        auto forward = split_chain[k1].get() == graph.get_chain_element(i, last);
        if (!forward)
        {
          k1 = Utils::increase(k, split_chain.size());
          if (split_chain[k1].get() != graph.get_chain_element(i, last))
            continue;
          ins = k1;
        }
        Topo::Wrap<Topo::Type::VERTEX> vv;
        vv.reset(const_cast<Topo::E<Topo::Type::VERTEX>*>(graph.get_chain_element(i, 1)));
        split_chain.insert(split_chain.begin() + ins, vv);
        _split_chains.emplace_back();
        auto& ch = _split_chains.back();
        if (forward)
          for (auto l = last + 1; l-- >= 0;)
            ch.push_back(Topo::Wrap<Topo::Type::VERTEX>(graph.get_chain_element(i, l)));
        else
          for (auto l = 0; l <= last; ++l)
            ch.push_back(Topo::Wrap<Topo::Type::VERTEX>(graph.get_chain_element(i, l)));
        for (auto vert : ch)
          std::remove(_verts.begin(), _verts.end(), vert);
        done = true;
      }
      if (done)
        break;
    }
  }
#endif
  return _verts.empty();
}

void FaceEdgeMap::check_doubious_faces()
{
  // Can happen that a chain belonging to a boundary of
  // an overlap is present two times on a face. This must be avoided.
  for (size_t i = 0; i < std::size(map_); ++i)
    for (auto& face_info : map_[i])
    {
      auto& vertsets = face_info.second.new_verts_;
      NewVerts::iterator vertsets_it_next;
      for (auto vertsets_it = vertsets.begin(); vertsets_it != vertsets.end(); 
        vertsets_it = vertsets_it_next)
      {
        vertsets_it_next = std::next(vertsets_it);
        if (!vertsets_it->doubious_)
          continue;
        bool to_remove = false;
        for (auto vertsets_it2 = vertsets.begin();
          !to_remove && vertsets_it2 != vertsets.end();
          ++vertsets_it2)
        {
          if (vertsets_it2->doubious_ || vertsets_it2 == vertsets_it)
            continue;
          to_remove = std::includes(vertsets_it2->begin(), vertsets_it2->end(),
            vertsets_it->begin(), vertsets_it->end());
        }
        if (to_remove)
          vertsets_it_next = vertsets.erase(vertsets_it);
      }
    }
}

bool FaceEdgeMap::split_on_boundary(
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX>& it_ev,
  bool first_is_common, bool prev_is_common,
  Topo::VertexChains& comm_verts, Topo::VertexChains& not_comm_verts,
  CommonVertices& edge_set_copy,
  OverlapFcesVectror& _overlap_faces,
  Topo::Wrap<Topo::Type::FACE>& face,
  Geo::Vector3& face_normal,
  Topo::VertexChains& split_chains
  )
{
  auto merge_tail = [&it_ev](Topo::VertexChains& _chain)
  {
    if (_chain.size() <= 1 || *it_ev.begin() != _chain[0][0])
      return;

    _chain.front().insert(
      _chain.front().begin(),
      _chain.back().begin(), _chain.back().end());
    _chain.pop_back();
  };
  if (first_is_common == prev_is_common)
  {
    if (prev_is_common)
      merge_tail(comm_verts);
    else
      merge_tail(not_comm_verts);
  }
  // We split the face assuming that the overlap is just with one face.
  // It should be true, nevertheless it would be better to 
  // test for this condition.
  if (not_comm_verts.empty() && edge_set_copy.empty())
  {
    _overlap_faces.push_back(face); // All vertices are in common.
    return false;
  }
  THROW_IF(not_comm_verts.empty(),
           "Faces without 'not common vertices' but not all common vertices are linked.");
  for (size_t j = 0; j < not_comm_verts.size(); ++j)
  {
    size_t j1 = j, j2 = j;
    if (first_is_common)
    {
      if (++j2 >= comm_verts.size())
        j2 = 0;
    }
    else
    {
      if (j1 > 0) --j1;
      else j1 = comm_verts.size() - 1;
    }
    not_comm_verts[j].insert(not_comm_verts[j].begin(), comm_verts[j1].back());
    not_comm_verts[j].push_back(comm_verts[j2].front());
  }
  split_chains.emplace_back();
  Topo::VertexChains connections;
  auto end_ch_vert = comm_verts.back().back();
  bool double_connection = false;
  for (auto& chain : comm_verts)
  {
    bool already_connected = false;
    for (const auto& conn : connections)
    {
      already_connected =
        conn.front() == end_ch_vert && conn.back() == chain.front();
      already_connected |=
        conn.front() == chain.front() && conn.back() == end_ch_vert;
      if (already_connected)
        break;
    }
    double_connection |= already_connected;
    connections.emplace_back();
    auto& curr_conn = connections.back();
    if (!Topo::connect_entities(end_ch_vert, chain.front(),
                                edge_set_copy, curr_conn, 
                                already_connected ? 2 : 0 ))
    {
      curr_conn.clear();
      curr_conn.push_back(end_ch_vert);
      curr_conn.push_back(chain.front());
    }
    end_ch_vert = chain.back();
    if (curr_conn.size() > 2)
      split_chains.back().insert(split_chains.back().end(),
                                 std::next(curr_conn.begin()), std::prev(curr_conn.end()));
    split_chains.back().insert(split_chains.back().end(), chain.begin(), chain.end());
  }
  if (double_connection && split_chains[0].size() >= 3)
  {
    if (face_normal == Geo::Vector3{ 0. })
      face_normal = Geo::vertex_polygon_normal(
        it_ev.begin(), it_ev.end());

    auto overlap_normal = Geo::vertex_polygon_normal(
      split_chains[0].begin(), split_chains[0].end());
    if (overlap_normal * face_normal < 0)
    {
      std::reverse(split_chains[0].begin(), split_chains[0].end());
      for (auto& conn : connections)
        std::reverse(conn.begin(), conn.end());
    }
  }
  for (auto& chain : not_comm_verts)
  {
    for (auto& conn : connections)
    {
      if (chain.front() == conn.front() && chain.back() == conn.back())
      {
        split_chains.emplace_back();
        split_chains.back().insert(
          split_chains.back().end(), chain.begin(), chain.end());
        split_chains.back().insert(split_chains.back().end(),
                                   std::next(conn.rbegin()), std::prev(conn.rend()));
        break;
      }
    }
  }
  if (!insert_remaning_common_vertices(
    edge_set_copy, connections, split_chains))
  {
    Topo::VertexChain::iterator it0, it1;
    auto j = split_chains.size();
    do
    {
      while (j-- > 0)
      {
        it0 = std::find(split_chains[j].begin(), split_chains[j].end(),
                        split_chains[0].back());
        if (it0 == split_chains[j].end())
          continue;
        auto start = split_chains[0].front() != split_chains[0].back() ?
          split_chains[j].begin() : std::next(it0);
        it1 = std::find(start, split_chains[j].end(),
                        split_chains[0].front());
        if (it1 == split_chains[j].end())
          continue;
        break;
      }
      THROW_IF(j == 0, "Insertion point not found");
      it1 = std::next(it1);
      if (it1 == split_chains[j].end())
        it1 = split_chains[j].begin();
    } while (it1 != it0);

#if 0
    while (!edge_set_copy.empty())
    {
      Geo::Point pt_end;
      split_chains[0].back()->geom(pt_end);
      auto min_it = std::min_element(edge_set_copy.begin(),
                                     edge_set_copy.end(), [&pt_end](
                                       const Topo::Wrap<Topo::Type::VERTEX>& _vert1,
                                       const Topo::Wrap<Topo::Type::VERTEX>& _vert2)
      {
        Geo::Point pt_curr1, pt_curr2;
        _vert1->geom(pt_curr1);
        _vert2->geom(pt_curr2);
        return Geo::length_square(pt_curr1 - pt_end) < Geo::length_square(pt_curr2 - pt_end);
      }
      );
      split_chains[0].push_back(*min_it);
      it1 = split_chains[j].insert(it1, *min_it);
      edge_set_copy.erase(min_it);
    }
#endif
    for (const auto& vert : edge_set_copy)
    {
      split_chains[0].push_back(vert);
      it1 = split_chains[j].insert(it1, vert);
    }
  }

#if 0
  if (split_chains.size() == 2 && split_chains[0].size() == vert_set.size())
  {
    // Can be an intersection on boundary. Let's check the normals.
    auto norm = Geo::vertex_polygon_normal(
      split_chains[0].begin(), split_chains[0].end());
    if (face_info.second.norm_ * norm < 0)
      continue; // The split is on the boundary.
  }
#endif
  return true;
}

void FaceEdgeMap::split_overlaps(OverlapFces&  _overlap_faces)
{
  // Process overlaps.
  for (size_t i = 0; i < std::size(map_); ++i)
  {
    for (auto& face_info : map_[i])
    {
      auto& edges = face_info.second.new_verts_;
      std::sort(edges.begin(), edges.end(), 
                [](const CommonVertices& _a, const CommonVertices& _b)
      { return _a.size() < _b.size(); });
      for (auto edge_it = edges.crbegin(); edge_it != edges.crend();
        ++edge_it)
      {
        auto& vert_set = *edge_it;
        if (vert_set.size() <= 2 || vert_set.doubious_)
          continue;
        // We have an overlap (more than 2 vertices in common with another face).
        auto& faces = face_info.second.new_faces_;
        int best_idx = 0;
        if (faces.size() > 1)
        {
          int best_score = 0;
          for (auto i_face = 0; i_face != faces.size(); ++i_face)
          {
            int face_score = 0;
            for (const auto& vert : vert_set)
            {
              Geo::Point pt;
              vert->geom(pt);
              face_score +=
                Topo::PointInFace::classify(faces[i_face], pt) !=
                Geo::PointInPolygon::Outside;
            }
            if (face_score > best_score)
            {
              best_score = face_score;
              best_idx = i_face;
            }
          }
        }
        if (best_idx < 0)
          continue;
        auto& face = faces[best_idx];
        auto edge_set_copy(vert_set);
        // Finds chains of common vertices on current face vertices.
        // Current face vertices are divided in chains of common vertices 
        // and not common vertices.
        Topo::VertexChains comm_verts, not_comm_verts;
        Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> it_ev(face);
        bool prev_is_common = false;
        bool first = true;
        bool first_is_common = false;
        Geo::Vector3 face_normal{ 0 };
        for (auto vert : it_ev)
        {
          auto it = std::find(edge_set_copy.begin(), edge_set_copy.end(), vert);
          if (it == edge_set_copy.end())
          {
            if (prev_is_common || not_comm_verts.empty())
              not_comm_verts.emplace_back();
            not_comm_verts.back().push_back(vert);
            prev_is_common = false;
          }
          else
          {
            edge_set_copy.erase(it);
            if (!prev_is_common)
              comm_verts.emplace_back();
            comm_verts.back().push_back(vert);
            prev_is_common = true;
            if (first)
              first_is_common = true;
          }
          first = false;
        }
        Topo::VertexChains split_chains;
        Topo::Split<Topo::Type::FACE> face_splitter(face);
        if (comm_verts.empty())
        {
          bool ok = false;
          if (edge_set_copy.size() > 1)
          {
            Topo::VertexChain edge_set_copy2(edge_set_copy.size() - 2);
            split_chains.emplace_back();
            for (auto it0 = edge_set_copy.begin();
                 it0 != edge_set_copy.end() && !ok; ++it0)
            {
              for (auto it1 = it0;
                   ++it1 != edge_set_copy.end() && !ok;)
              {
                split_chains[0].clear();
                std::copy_if(edge_set_copy.begin(), edge_set_copy.end(),
                             edge_set_copy2.begin(),
                             [&it1, &it0](const Topo::Wrap<Topo::Type::VERTEX>& _vert)
                {
                  return _vert != *it0 && _vert != *it1;
                });
                ok = Topo::connect_entities(
                  *it0, *it1, edge_set_copy2, split_chains[0], 2) &&
                  split_chains[0].size() == edge_set_copy.size();
              }
            }
          }
          if (!ok)
            continue;
          face_splitter.use_face_loops();
          for (auto& split_chain : split_chains)
            face_splitter.add_island(split_chain);
        }
        else
        {
          if (!split_on_boundary(
            it_ev, first_is_common, prev_is_common,
            comm_verts, not_comm_verts, edge_set_copy,
            _overlap_faces[i],
            face, face_normal, split_chains))
          {
            continue;
          }
          for (auto& split_chain : split_chains)
            face_splitter.add_boundary(split_chain);
        }

        face_splitter.compute();
        auto& new_faces = face_splitter.new_faces();
        _overlap_faces[i].push_back(new_faces[0]);
        faces.erase(faces.begin() + best_idx);
        faces.insert(faces.end(), std::next(new_faces.begin()), new_faces.end());
      }
    }
  }
}


// Given the chains of vertices to be used as new vertices for the split faces,
// and the vertices _vert_a and _vert_b where the split part split the face, solves
// ambiguous split where _vert_a and/or _vert_b may appear more than once in some
// of the new chains of vertices. If this happens this means that the face has 
// some internal loop attached to a bundary. The following algorithm split the
// loops at these vertices, classifies the loops as internal or external and finally
// moves all the internal loop in the appropriate external loop.
void resolve_ambiguities(
  Topo::VertexChains& _split_chains,
  Topo::Wrap<Topo::Type::VERTEX> _vert_a, Topo::Wrap<Topo::Type::VERTEX> _vert_b,
  const Geo::Point& _expct_norm)
{
  Topo::Wrap<Topo::Type::VERTEX>* test_verts[2] = { &_vert_a, &_vert_b };
  std::array<std::vector<std::vector<size_t>>, 2> vert_mults;
  for (auto& vert_mult : vert_mults)
    vert_mult.resize(_split_chains.size());

  for (size_t i_vert = 0; i_vert < std::size(test_verts); ++i_vert)
  {
    for (size_t i_ch = 0; i_ch < _split_chains.size(); ++i_ch)
    {
      for (size_t i_ch_vert = 0; i_ch_vert < _split_chains[i_ch].size(); ++i_ch_vert)
      {
        if (_split_chains[i_ch][i_ch_vert] == *test_verts[i_vert])
          vert_mults[i_vert][i_ch].push_back(i_ch_vert);
      }
    }
  }

  for (size_t i_vert = 0; i_vert < std::size(test_verts); ++i_vert)
  {
    std::vector<Topo::VertexChains> ch_segs_external(_split_chains.size());
    Topo::VertexChains ch_segs_internal;

    for (size_t i_ch = 0; i_ch < _split_chains.size(); ++i_ch)
    {
      if (vert_mults[i_vert][i_ch].size() < 2)
        continue;
      auto start = std::prev(vert_mults[i_vert][i_ch].end());
      for (auto end = vert_mults[i_vert][i_ch].begin();
        end != vert_mults[i_vert][i_ch].end(); start = end++)
      {
        Topo::VertexChain curr_chain;
        for (size_t i = *start; i != *end; )
        {
          curr_chain.push_back(_split_chains[i_ch][i]);
          if (++i >= _split_chains[i_ch].size())
            i = 0;
        }
        auto norm = make_polygonal_face(curr_chain)->normal();
        if (_expct_norm * norm < 0)
          ch_segs_internal.emplace_back(std::move(curr_chain));
        else
          ch_segs_external[i_ch].emplace_back(std::move(curr_chain));
      }
    }
    if (ch_segs_internal.empty())
      continue;
    for (auto& chain : ch_segs_internal)
    {
      Geo::Point pt;
      chain[1]->geom(pt);
      if (chain.size() == 1)
      {
        Geo::Point pt0;
        chain[0]->geom(pt0);
        pt = (pt + pt0) * 0.5;
      }
      bool located = false;
      for (size_t i_ch = 0; i_ch < _split_chains.size() && !located; ++i_ch)
      {
        for (auto& chain_ext : ch_segs_external[i_ch])
        {
          if (point_in_polygon(chain_ext, pt, &_expct_norm) == Geo::PointInPolygon::Inside)
          {
            ch_segs_external[i_ch].emplace_back(std::move(chain));
            located = true;
            break;
          }
        }
      }
      THROW_IF(!located, "Reversed loop not loacated!")
    }
    for (size_t i_ch = 0; i_ch < _split_chains.size(); ++i_ch)
    {
      _split_chains[i_ch].clear();
      for (auto& achain : ch_segs_external[i_ch])
      {
        _split_chains[i_ch].insert(_split_chains[i_ch].end(), achain.begin(), achain.end());
      }
    }
  }
}

namespace {
void add_open_chain(
  const Topo::Wrap<Topo::Type::VERTEX>& _vert, 
  EdgeChain & edge_ch,
  Topo::Wrap<Topo::Type::FACE> _face, 
  FaceEdgeMap::NewVerts& _edge_vec)
{
  if (edge_ch.empty())
    return;
  auto pos = _face->find_child(_vert.get());
  if (pos == SIZE_MAX)
    return;
  for (auto v_ch : edge_ch)
    _face->insert_child((*v_ch)[1].get(), ++pos);
  ++pos;
  for (auto v_ch : edge_ch)
  {
    _face->insert_child((*v_ch)[0].get(), pos);
    v_ch->clear();
  }

  _edge_vec.erase(
    std::remove_if(_edge_vec.begin(), _edge_vec.end(),
      [](const FaceEdgeMap::CommonVertices& _comm_v) { return _comm_v.empty(); }),
    _edge_vec.end());
}

}

void FaceEdgeMap::split_with_chains()
{
  auto clean_edge_vec = [](NewVerts& _edge_vec, const EdgeChain& _from_edge_ch)
  {
    // Remove the used edges from the set of new edges.
    for (auto ch : _from_edge_ch) ch->clear();
    _edge_vec.erase(
      std::remove_if(_edge_vec.begin(), _edge_vec.end(),
        [](const CommonVertices& _comm_v) { return _comm_v.empty(); }),
      _edge_vec.end());
  };
  for (size_t i = 0; i < std::size(map_); ++i)
  {
    for (auto& face_info : map_[i])
    {
      auto& faces = face_info.second.new_faces_;
      auto& edge_vec = face_info.second.new_verts_;
      if (edge_vec.empty())
        continue;
      // Try face split.
      for (size_t j = 0; j < faces.size() && !edge_vec.empty(); ++j)
      {
        auto& face = faces[j];
        Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(face);
        auto prev = *fv_it.begin();
        for (auto f_it = fv_it.end(); f_it-- != fv_it.begin(); prev = *f_it)
        {
          for (auto ed_it = edge_vec.begin(); ed_it != edge_vec.end(); )
          {
            if (ed_it->size() == 2 &&
              (prev == (*ed_it)[0] && *f_it == (*ed_it)[1]) ||
              (prev == (*ed_it)[1] && *f_it == (*ed_it)[0]))
            {
              ed_it = edge_vec.erase(ed_it);
            }
            else
              ++ed_it;
          }
        }
        if (edge_vec.empty())
          break;
        for (auto vert_it = fv_it.begin(); vert_it != fv_it.end(); )
        {
          EdgeChain edge_ch;
          Topo::Wrap<Topo::Type::VERTEX>* last_vert_it;
          if (!find_edge_chain(*vert_it, edge_vec, fv_it, edge_ch, last_vert_it))
          {
            add_open_chain(*vert_it, edge_ch, face, edge_vec);
            ++vert_it; // Try next vertex.
            continue;
          }

          Topo::VertexChains split_chains;
          split_chains.resize(2);
          split_chains[0].push_back((*edge_ch[0])[0]);
          for (auto& edge_it : edge_ch)
            split_chains[0].push_back((*edge_it)[1]);

          if (*vert_it == *last_vert_it)
          {
            // Closed loop inside face starting from one vertex.
            // The chain sense is ambiguos.
            auto norm = face_info.second.norm_;
            auto chain_normal = Geo::vertex_polygon_normal(
              split_chains[0].begin(), split_chains[0].end());
            if (norm * chain_normal > 0)
              std::reverse(split_chains[0].begin(), split_chains[0].end());
          }

          split_chains[1] = split_chains[0];

          Topo::Wrap<Topo::Type::VERTEX>* first_vert_it = vert_it;
          std::reverse(split_chains[1].begin(), split_chains[1].end());
          auto complete_chain = [&fv_it](Topo::Wrap<Topo::Type::VERTEX>* _first,
            Topo::Wrap<Topo::Type::VERTEX>* _last, Topo::VertexChain& _chain)
          {
            auto v_it = _first;
            while (++v_it != _last && v_it != fv_it.end())
              _chain.push_back(*v_it);
            if (v_it == _last)
              return;
            for (v_it = fv_it.begin(); v_it != _last; ++v_it)
              _chain.push_back(*v_it);
          };
          if (last_vert_it != first_vert_it)
            complete_chain(first_vert_it, last_vert_it, split_chains[1]);
          else
            split_chains[1].pop_back(); // Last element is duplicated.
          complete_chain(last_vert_it, first_vert_it, split_chains[0]);

          //auto norm = std::get<Normal>(face_info.second);
          //resolve_ambiguities(
          //  split_chains, (*edge_ch.front())[0], (*edge_ch.back())[1], norm);

          Topo::Split<Topo::Type::FACE> face_splitter(face);
          for (auto& split_chain : split_chains)
            face_splitter.add_boundary(split_chain);
          face_splitter.compute();
          auto& new_fa = face_splitter.new_faces();
          if (!new_fa.empty())
          {
            face = face_splitter.new_faces()[0];
            faces.insert(faces.end(), new_fa.cbegin() + 1, new_fa.cend());
          }
          // Remove the used edges from the set of new edges.
          clean_edge_vec(edge_vec, edge_ch);
          // We have performed one split. Lets re-process the same face and all the others.
          --j;
          break;
        }
      }
      while (!edge_vec.empty())
      {
        auto ed = edge_vec.back();
        EdgeChain edge_ch;
        Topo::Wrap<Topo::Type::VERTEX>* last_vert_it;
        auto& cur_face = faces.front();
        Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(cur_face);
        if (!find_edge_chain(ed.front(), edge_vec, fv_it, edge_ch, last_vert_it))
        {
          edge_vec.pop_back();
          continue;
          //add_open_chain(*vert_it, edge_ch, face, edge_vec);
        }
        Topo::VertexChain split_chain;
        for (auto& ed_it : edge_ch)
          split_chain.push_back((*ed_it)[0]);
        Topo::Split<Topo::Type::FACE> face_splitter(cur_face);
        face_splitter.add_island(split_chain);
        face_splitter.use_face_loops();
        face_splitter.compute();
        auto& new_fa = face_splitter.new_faces();
        if (!new_fa.empty())
        {
          faces[0] = face_splitter.new_faces()[0];
          faces.insert(faces.end(), new_fa.cbegin() + 1, new_fa.cend());
        }
        clean_edge_vec(edge_vec, edge_ch);
      }
      if (!edge_vec.empty())
      {
        //std::cout << edge_vec.size() << "::::\n";
        for (auto ff : faces)
        {
          //std::cout << ff->id() << std::endl;
          THROW_IF(!edge_vec.empty(), "Vector edge not empty.");
        }
      }
    }
  }
}

void FaceEdgeMap::split(OverlapFces& _overlap_faces)
{
  check_doubious_faces();
  split_overlaps(_overlap_faces);
  split_with_chains();
}

}//namespace

}//namespace Boolean
