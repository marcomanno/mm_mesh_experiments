#pragma optimize("", off)
#include "face_intersections.hh"
#include "Geo/plane_fitting.hh"
#include "Geo/vector.hh"
#include <Geo/point_in_polygon.hh>
#include <Topology/impl.hh>
#include <Topology/shared.hh>
#include <Topology/split.hh>
#include "Topology/connect.hh"
#include "Utils/error_handling.hh"

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

struct FaceEdgeMap
{
  typedef std::vector<Topo::Wrap<Topo::Type::VERTEX>> CommonVertices;
  typedef std::vector<CommonVertices> NewVerts;
  typedef std::vector<Topo::Wrap<Topo::Type::FACE>> NewFaces;

  bool add_face_edge(
    const Topo::Wrap<Topo::Type::FACE>& _face,
    const std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _v_inters,
    bool _is_face_b)
  {
    auto& map = map_[_is_face_b];
    auto elem = map.emplace(_face, FaceData());

    auto& data = elem.first->second;
    std::get<NewVerts>(data).push_back(_v_inters);

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
      auto pl_geom = Geo::IPolygonalFace::make(face_pts.begin(), face_pts.end());
      if (pl_geom->normal() * n < 0)
        n = -n;
      std::get<Normal>(data) = n;
    }
    return true;
  }

  void init_map();
  void split(OverlapFces& _overlap_faces);

private:
  void split_overlaps_on_boundary(OverlapFces&  _overlap_faces);
  void split_with_chains();
  typedef Geo::Point Normal;

  typedef std::tuple<Normal, NewFaces, NewVerts> FaceData;
  std::map<Topo::Wrap<Topo::Type::FACE>, FaceData> map_[2];
};

} //namespace


bool FaceVersus::face_intersect(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_a,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it_b)
{
  FaceEdgeMap face_new_edge_map;
  for (auto face_a : _face_it_a)
  {
    auto vert_set_a = face_vertices(f_vert_info_[face_a].new_vert_list_, face_a);
    for (auto face_b : _face_it_b)
    {
      const auto& vert_set_b = 
        face_vertices(f_vert_info_[face_b].new_vert_list_, face_b);
      std::vector<Topo::Wrap<Topo::Type::VERTEX>> v_inters;
      std::set_intersection(
        vert_set_a.begin(), vert_set_a.end(),
        vert_set_b.begin(), vert_set_b.end(),
        std::back_inserter(v_inters));
      if (v_inters.size() < 2)
        continue;

      face_new_edge_map.add_face_edge(face_a, v_inters, false);
      face_new_edge_map.add_face_edge(face_b, v_inters, true);
    }
  }
  face_new_edge_map.init_map();
  face_new_edge_map.split(overlap_faces_);
  return true;
}

namespace {

void FaceEdgeMap::init_map()
{
  for (auto& map : map_)
  {
    for (auto& face_info : map)
      std::get<NewFaces>(face_info.second).push_back(face_info.first);
  }
}

void FaceEdgeMap::split_overlaps_on_boundary(OverlapFces&  _overlap_faces)
{
  // Process overlaps.
  for (size_t i = 0; i < std::size(map_); ++i)
  {
    for (auto& face_info : map_[i])
    {
      auto& edges = std::get<NewVerts>(face_info.second);
      NewVerts::iterator edge_next;
      for (auto edge_it = edges.begin(); edge_it != edges.end();
        edge_it = edge_next)
      {
        edge_next = std::next(edge_it);
        auto& vert_set = *edge_it;
        if (vert_set.size() <= 2)
          continue;
        // We have an overlap (more than 2 vertices in common with another face).
        auto& faces = std::get<NewFaces>(face_info.second);
        for (auto i_face = 0; i_face != faces.size(); ++i_face)
        {
          auto& face = faces[i_face];
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
          if (comm_verts.empty())
            continue;
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
            _overlap_faces[i].push_back(face); // All vertices are in common.
            continue;
          }
          THROW_IF(not_comm_verts.empty(),
            "Faces without not common vertices but not "
            "all common vertices are linked.");
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
          Topo::VertexChains split_chains;
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
              edge_set_copy, curr_conn, !already_connected))
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
          if (double_connection)
          {
            if (face_normal == Geo::Vector3{0.})
              face_normal = Geo::get_polygon_normal(
                it_ev.begin(), it_ev.end());

            auto overlap_normal = Geo::get_polygon_normal(
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
          if (!edge_set_copy.empty())
          {
            Topo::VertexChain::iterator it0, it1;
            auto j = split_chains.size();
            do {
              while (j > 0)
              {
                --j;
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

            for (const auto& vert : edge_set_copy)
            {
              split_chains[0].push_back(vert);
              it1 = split_chains[j].insert(it1, vert);
            }
          }

          Topo::Split<Topo::Type::FACE> face_splitter(face);
          face_splitter(split_chains);
          auto& new_faces = face_splitter.new_faces();
          _overlap_faces[i].push_back(new_faces[0]);
          faces.erase(faces.begin() + i_face);
          faces.insert(faces.end(), std::next(new_faces.begin()), new_faces.end());
          edge_next = edges.erase(edge_it);
          break;
        }
      }
    }
  }
}

typedef std::vector<FaceEdgeMap::NewVerts::iterator> EdgeChain;

// Finds a set of connected edges (couple of vertices) and using the Edges in _pt_sets
bool find_edge_chain(
  const Topo::Wrap<Topo::Type::VERTEX>& _vert,
  FaceEdgeMap::NewVerts& _ed_sets,
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX>& _fv_it,
  EdgeChain& _edge_ch,
  Topo::Wrap<Topo::Type::VERTEX>*& _last_vert)
{
  auto follow_chain = [&_ed_sets,&_fv_it, &_last_vert](
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
    if (Geo::PointInPolygon::classify(polygon, pt) == Geo::PointInPolygon::Inside)
      return true;
    else
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

      EdgeChain edge_ch;
      if (follow_chain(edge_itr, edge_ch))
      {
        _edge_ch = std::move(edge_ch);
        return true;
      }
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
  return Geo::IPolygonalFace::make(pts.begin(), pts.end());
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

void FaceEdgeMap::split_with_chains()
{
  for (size_t i = 0; i < std::size(map_); ++i)
  {
    for (auto& face_info : map_[i])
    {
      auto& faces = std::get<NewFaces>(face_info.second);
      auto& edge_vec = std::get<NewVerts>(face_info.second);
      for (size_t j = 0; j < faces.size() && !edge_vec.empty(); ++j)
      {
        auto& face = faces[j];
        Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(face);
        for (auto vert_it = fv_it.begin(); vert_it != fv_it.end(); )
        {
          EdgeChain edge_ch;
          Topo::Wrap<Topo::Type::VERTEX>* last_vert_it;
          if (!find_edge_chain(*vert_it, edge_vec, fv_it, edge_ch, last_vert_it))
          {
            ++vert_it; // Try next vertex.
            continue;
          }

          Topo::VertexChains split_chains;
          split_chains.resize(2);
          split_chains[0].push_back((*edge_ch[0])[0]);
          for (auto& edge_it : edge_ch)
            split_chains[0].push_back((*edge_it)[1]);

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
          face_splitter(split_chains);
          auto& new_fa = face_splitter.new_faces();
          if (!new_fa.empty())
          {
            face = face_splitter.new_faces()[0];
            faces.insert(faces.end(), new_fa.cbegin() + 1, new_fa.cend());
          }
          // Remove the used edges from the set of new edges.
          for (auto ch : edge_ch) ch->clear();
          edge_vec.erase(
            std::remove_if(edge_vec.begin(), edge_vec.end(),
              [](const CommonVertices& _comm_v) { return _comm_v.empty(); }),
            edge_vec.end());
          // We have performed one split. Lets re-process the same face and all the others.
          --j;
          break;
        }
      }
    }
  }
}

void FaceEdgeMap::split(OverlapFces& _overlap_faces)
{
  split_overlaps_on_boundary(_overlap_faces);
  split_with_chains();
}

}//namespace

}//namespace Boolean
