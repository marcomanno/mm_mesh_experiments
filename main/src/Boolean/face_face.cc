//#pragma optimize ("", off)
#include "face_intersections.hh"
#include "Base/basic_type.hh"
#include "Geo/kdtree.hh"
#include "Geo/plane_fitting.hh"
#include "Geo/vector.hh"
#include <Geo/point_in_polygon.hh>
#include "Import/import.hh"
#include "Topology/connect.hh"
#include "Topology/geom.hh"
#include "Topology/impl.hh"
#include "Topology/shared.hh"
#include "Topology/split.hh"
#include <Topology/split_chain.hh>
#include "Utils/error_handling.hh"
#include "Utils/circular.hh"
#include "Utils/graph.hh"

#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/adaptor/copied.hpp>

#include <list>
#include <set>

namespace Boolean {

namespace {

static std::set<Topo::Wrap<Topo::Type::VERTEX>> face_vertices(
  const std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _verts,
  const Topo::Wrap<Topo::Type::FACE>& _face_a)
{
  std::set<Topo::Wrap<Topo::Type::VERTEX>> vert_set;
  vert_set.insert(_verts.begin(), _verts.end());
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(_face_a);
  vert_set.insert(fv_it.begin(), fv_it.end());
  return vert_set;
}

using Connection = std::array<Topo::Wrap<Topo::Type::VERTEX>, 2>;

static bool boundary_chain(
  const Topo::Wrap<Topo::Type::FACE>& _face,
  const std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _v_inters,
  Connection& _start_end)
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

static bool 
mid_point_in_face(const Connection& _start_end,
                  const Topo::Wrap<Topo::Type::FACE>& _face)
{
  Geo::VectorD3 mid_pt = { 0 };
  for (int i = 0; i < 2; ++i)
  {
    Geo::Point pt;
    _start_end[i]->geom(pt);
    mid_pt += pt;
  }
  mid_pt /= 2.;
  return Topo::PointInFace::classify(_face, mid_pt) == 
    Geo::PointInPolygon::Inside;
}

struct FaceEdgeMap
{
  struct CommonVertices : public Topo::VertexChain
  {
    CommonVertices(const Topo::VertexChain& _comm_vert, bool _doubious) :
      Topo::VertexChain(_comm_vert), doubious_(_doubious) {}
    bool doubious_ = false;
  };
  typedef std::vector<CommonVertices> NewVerts;
  typedef std::vector<Topo::Wrap<Topo::Type::FACE>> NewFaces;
  struct FaceData
  {
    Geo::VectorD3 norm_;
    NewFaces new_faces_;
    NewVerts new_verts_;
  };
  typedef std::map<Topo::Wrap<Topo::Type::FACE>, FaceData> FaceDataMap;

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
  FaceDataMap map_[2];
  void check_doubious_faces();
};
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

    Connection start_end_a, start_end_b;
    auto add_a = !boundary_chain(face_a, v_inters, start_end_a);
    auto add_b = !boundary_chain(face_b, v_inters, start_end_b);
    if (v_inters.size() > 2 && add_a != add_b)
    {
      if (!add_a)
        add_a = add_b = mid_point_in_face(start_end_a, face_a);
      else
        add_a = add_b = mid_point_in_face(start_end_b, face_b);
    }
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

static void split_face(
  FaceEdgeMap::FaceDataMap::value_type& face_info,
  OverlapFcesVectror& _overlap_faces)
{
  auto& edge_vec = face_info.second.new_verts_;
  if (edge_vec.empty())
    return;
  auto make_connection = [](
    const Topo::Wrap<Topo::Type::VERTEX>& _a,
    const Topo::Wrap<Topo::Type::VERTEX>& _b)
  {
    Connection conn = { _a, _b };
    std::sort(conn.begin(), conn.end());
    return conn;
  };
  auto ch_spliter = Topo::ISplitChain::make();
  auto face = face_info.first;
  Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> fl_it(face);
  std::set<Connection> existing_conn;
  for (auto loop : fl_it)
  {
    Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> lv_it(loop);
    auto prev_vert = *std::prev(lv_it.end());
    Topo::VertexChain ch;
    for (auto vert : lv_it)
    {
      existing_conn.insert(make_connection(prev_vert, vert));
      ch.push_back(vert);
      prev_vert = vert;
    }
    ch_spliter->add_chain(ch);
  }
  for (size_t i = 0; i < edge_vec.size(); ++i)
  {
    auto& vert_set = edge_vec[i];
    std::sort(vert_set.begin(), vert_set.end());
    auto last = std::prev(vert_set.end());
    for (auto vert_set_it1 = vert_set.begin();
         vert_set_it1 != last; ++vert_set_it1)
    {
      for (auto vert_set_it2 = vert_set_it1;
           ++vert_set_it2 != vert_set.end();)
      {
        auto conn = make_connection(*vert_set_it1, *vert_set_it2);
        if (existing_conn.find(conn) != existing_conn.end())
          continue;
        if (vert_set.size() == 2)
        {
          ch_spliter->add_connection(conn[0], conn[1]);
          existing_conn.insert(conn);
        }
        else
        {
          auto edges =
            Topo::shared_entities<Topo::Type::VERTEX, Topo::Type::EDGE>(conn[0], conn[1]);
          if (!edges.empty())
          {
            ch_spliter->add_connection(conn[0], conn[1]);
            existing_conn.insert(conn);
          }
        }
      }
    }
  }
  for (auto& vert_set : edge_vec)
  {
    if (vert_set.size() <= 2)
      continue;
    auto vert_set_cpy(vert_set);
    std::vector<std::vector<Topo::Wrap<Topo::Type::VERTEX>>> grp_grp_vert;
    while (!vert_set_cpy.empty())
    {
      grp_grp_vert.emplace_back();
      auto& curr = grp_grp_vert.back();
      auto move_vertex = [&curr, &vert_set_cpy](decltype(vert_set_cpy)::iterator _it)
      {
        curr.push_back(*_it);
        vert_set_cpy.erase(_it);
      };
      move_vertex(vert_set_cpy.begin());
      for (size_t i = 0; i < curr.size(); ++i)
      {
        for (size_t j = 0; j < vert_set_cpy.size();)
        {
          auto& elem = curr[i];
          auto conn = make_connection(elem, vert_set_cpy[j]);
          if (existing_conn.find(conn) != existing_conn.end())
            move_vertex(vert_set_cpy.begin() + j);
          else
            ++j;
        }
      }
    }
    if (grp_grp_vert.size() < 2)
      continue;
    if (grp_grp_vert.size() > 2)
      std::cout << "grp_grp_vert.size() > 2 in face split";
    [&] // This is a function used to return directly inside a double loop
    {
      std::vector<Connection> valid_conn;
      for (const auto& vv0 : grp_grp_vert[0])
        for (const auto& vv1 : grp_grp_vert[1])
        {
          auto conn = make_connection(vv0, vv1);
          if (!mid_point_in_face(conn, face))
            continue;
          auto res = ch_spliter->check_new_connection(conn[0], conn[1]);
          if (res == Topo::ISplitChain::ConnectionCheck::EXISTING)
            return;
          if (res == Topo::ISplitChain::ConnectionCheck::INVALID)
            continue;
          if (res == Topo::ISplitChain::ConnectionCheck::OK)
            valid_conn.push_back(conn);
        }
      if (valid_conn.empty())
        std::cout << "Not good";
      else
      {
        auto& conn = valid_conn[0];
        ch_spliter->add_connection(conn[0], conn[1]);
        existing_conn.insert(conn);
      }
    }();
  }
  ch_spliter->compute();
  Topo::Split<Topo::Type::FACE> fsplit(face);
  for (size_t i = 0; i < ch_spliter->boundaries().size(); ++i)
  {
    fsplit.add_boundary(ch_spliter->boundaries()[i]);
    auto isles = ch_spliter->boundary_islands(i);
    if (isles == nullptr)
      continue;
    for (const auto& isle : *isles)
      fsplit.add_original_island(isle);
  }
  fsplit.compute();
  const auto& newfaces = face_info.second.new_faces_ = fsplit.new_faces();
  std::sort(edge_vec.begin(), edge_vec.end());
  for (auto f : newfaces)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fe(f);
    Topo::VertexChain vc;
    for (auto v : fe)
      vc.push_back(v);
    std::sort(vc.begin(), vc.end());
    auto where = 
      std::equal_range(edge_vec.begin(), edge_vec.end(), vc);
    if (where.first != where.second)
      _overlap_faces.push_back(f);
  }
}

void FaceEdgeMap::split(OverlapFces& _overlap_faces)
{
  check_doubious_faces();
  for (size_t i = 0; i < std::size(map_); ++i)
    for (auto& face_info : map_[i])
      split_face(face_info, _overlap_faces[i]);
}

}//namespace

}//namespace Boolean
