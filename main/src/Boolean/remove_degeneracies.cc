
#pragma optimize ("", off)
#include "priv.hh"

#include <Geo/point_in_polygon.hh>
#include "Geo/pow.hh"

#include "Topology/impl.hh"
#include "Topology/merge.hh"
#include "Topology/shared.hh"
#include "Topology/split.hh"
#include "Utils/enum.hh"



namespace Boolean {

namespace {

const int SAFE_TOL_MULTIPLE_SQ = 3;

void remove_degenerate_edges(Topo::Wrap<Topo::Type::BODY>& _body)
{
  for (bool achange = true; achange; )
  {
    achange = false;
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> it_ed(_body);
    for (auto edge : it_ed)
    {
      // Removes out degenerate edges.
      Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> vv(edge);
      if (vv.get(0) == vv.get(1))
        continue;
      Geo::Segment seg;
      edge->geom(seg);
      auto tol = edge->tolerance();
      if (Geo::length_square(seg[1] - seg[0]) >
        SAFE_TOL_MULTIPLE_SQ * Geo::sq(tol))
        continue;
      Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_ve(edge);
      achange |= Topo::merge(it_ve.get(0), it_ve.get(1));
    }
  }
}

MAKE_ENUM(DegType, NONE, SPLITA, MERGE, SPLITB);

DegType degenerate_triangle(
  const Topo::Wrap<Topo::Type::VERTEX> _verts[3])
{
  if (_verts[0] == _verts[2] || _verts[0] == _verts[1] || 
    _verts[1] == _verts[2])
    return DegType::NONE;
  Geo::Point pt[3];
  double tol = 0;
  for (auto i : { 0, 1, 2 })
  {
    _verts[i]->geom(pt[i]);
    tol = std::max(tol, _verts[i]->tolerance());
  }
  auto vec0 = pt[0] - pt[1];
  auto vec1 = pt[2] - pt[1];
  if (vec0 * vec1 < 0)
    return DegType::NONE;
  auto len0_sq = Geo::length_square(vec0);
  auto len1_sq = Geo::length_square(vec1);
  auto h_sq = 
    Geo::length_square(vec0 % vec1) / std::max(len0_sq, len1_sq);
  if (h_sq > SAFE_TOL_MULTIPLE_SQ * Geo::sq(tol))
    return DegType::NONE;
  if (fabs(sqrt(len0_sq) - sqrt(len1_sq)) < sqrt(SAFE_TOL_MULTIPLE_SQ) * tol)
    return DegType::MERGE;
  return len0_sq > len1_sq ? DegType::SPLITA : DegType::SPLITB;
}

void remove_degenerate_triangles(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::LOOP>& it_loop)
{
  bool achange = false;
  for (auto loop : it_loop)
  {
    Topo::Iterator<Topo::Type::LOOP, Topo::Type::COEDGE> lc_it(loop);
    if (lc_it.size() < 3)
      continue;
    auto& last_coed = *std::prev(lc_it.end());
    Topo::Iterator<Topo::Type::COEDGE, Topo::Type::VERTEX> cv_it(last_coed);
    Topo::Wrap<Topo::Type::VERTEX> verts[3];
    verts[0] = *(cv_it.begin());
    verts[1] = *(std::next(cv_it.begin()));
    for (auto coed : lc_it)
    {
      cv_it.reset(coed);
      verts[2] = *(std::next(cv_it.begin()));
      switch (degenerate_triangle(verts))
      {
      case DegType::MERGE:
        achange |= Topo::merge(verts[0], verts[2]);
        break;
      case DegType::SPLITA:
        achange |= Topo::split(verts[0], verts[1], verts[2]);
        break;
      case DegType::SPLITB:
        achange |= Topo::split(verts[1], verts[2], verts[0]);
        break;
      }
      std::copy(verts + 1, verts + 3, verts);
    }
  }
}

bool external_spike(const Topo::Wrap<Topo::Type::LOOP>& _face,
  const Topo::Wrap<Topo::Type::EDGE>& _ed)
{
  Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev_it(_ed);
  Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> fv_it(_face);
  int dupl[2] = { 0, 0 };
  size_t ins = SIZE_MAX;
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> other_vertices;
  for (auto v : fv_it)
  {
    int i = 0;
    for (; i < 2 && v != ev_it.get(i); ++i);
    if (i >= 2)
      other_vertices.push_back(v);
    else
    {
      ++dupl[i];
      if (ins == SIZE_MAX)
      {
        ins = other_vertices.size();
        other_vertices.emplace_back();
      }
    }
  }
  if (ins >= other_vertices.size())
    return false;
  size_t i = dupl[0] > dupl[1] ? 0 : 1;
  other_vertices[ins] = ev_it.get(i);
  Geo::Point deg_pt;
  ev_it.get(1 - i)->geom(deg_pt);
  std::vector<Geo::Point> polygon(other_vertices.size());
  for(i = 0; i < other_vertices.size(); ++i)
    other_vertices[i]->geom(polygon[i]);

  auto pt_class = Geo::PointInPolygon::classify(polygon, deg_pt);
  return pt_class == Geo::PointInPolygon::Outside;
}

bool unique_body_faces(Topo::Wrap<Topo::Type::EDGE> _ed)
{
  Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> ef_it(_ed);
  auto it = ef_it.begin();
  auto b = (*it)->get(Topo::Direction::Up, 0);
  while (++it != ef_it.end())
    if (b != (*it)->get(Topo::Direction::Up, 0))
      return false;
  return true;
};

}//namespace

// Removes coincident consecutive vertices in face loops.
// Removes faces with less than 3 vertices.
bool remove_degeneracies(Topo::Wrap<Topo::Type::BODY>& _body)
{
  remove_degenerate_edges(_body);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::LOOP> it_loop(_body);
  remove_degenerate_triangles(it_loop);
  bool achange = false;
  for (auto loop : it_loop)
  {
    if (!loop.get())
      continue;
    bool loc_change;
    do {
      loc_change = false;
      auto n_verts = loop->size(Topo::Direction::Down);
      if (n_verts >= 3)
      {
        auto prev_vert_ptr = loop->get(Topo::Direction::Down, 0);
        for (size_t j = n_verts; j-- > 0; )
        {
          auto curr_vert_ptr = loop->get(Topo::Direction::Down, j);
          if (curr_vert_ptr == prev_vert_ptr)
          {
            loc_change = true;
            loop->remove_child(j); // curr_vert_ptr->remove();
          }
          else
            prev_vert_ptr = curr_vert_ptr;
        }
      }
      Topo::Iterator<Topo::Type::LOOP, Topo::Type::COEDGE> fc_it(loop);
      if (fc_it.size() >= 4)
      {
        Topo::Iterator<Topo::Type::COEDGE, Topo::Type::EDGE> 
          prev_ed_it(*std::prev(fc_it.end()));
        auto prev_ed = *prev_ed_it.begin();
        std::vector<Topo::Wrap<Topo::Type::VERTEX>> rem_verts;
        for (auto ced : fc_it)
        {
          Topo::Iterator<Topo::Type::COEDGE, Topo::Type::EDGE> ed_it(ced);
          auto ed = *ed_it.begin();
          if (ed == prev_ed)
          {
            if (unique_body_faces(ed) || external_spike(loop, ed))
            {
              Topo::Iterator<Topo::Type::COEDGE, Topo::Type::VERTEX> cv_it(ced);
              rem_verts.push_back(cv_it.get(0));
            }
          }
          prev_ed = ed;
        }
        for (auto v : rem_verts)
          loop->remove_child(v.get());
        loc_change |= !rem_verts.empty();
      }
      achange |= loc_change;
    } while (loc_change);
    if (loop->size(Topo::Direction::Down) < 3)
    {
      achange |= true;
      loop->remove();
    }
  }
  return achange;
}

}