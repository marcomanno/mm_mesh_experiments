
#include "priv.hh"

#include <Geo/point_in_polygon.hh>
#include "Geo/pow.hh"

#include "Topology/impl.hh"
#include "Topology/merge.hh"
#include "Topology/shared.hh"
#include "Topology/split.hh"
#include "Utils/enum.hh"

#include <set>

namespace Boolean {

namespace {

const int SAFE_TOL_MULTIPLE_SQ = 4;

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
      auto len2 = Geo::length_square(seg[1] - seg[0]);
      if (len2 > SAFE_TOL_MULTIPLE_SQ * Geo::sq(tol))
        continue;
      Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_ve(edge);
      achange |= Topo::merge(it_ve.get(0), it_ve.get(1));
    }
  }
}

MAKE_ENUM(DegType, NONE, SPLITA, MERGE, SPLITB);

DegType degenerate_triangle(
  const Topo::Wrap<Topo::Type::VERTEX> _verts[3], double& _dist_sq)
{
  if (_verts[0] == _verts[2] || _verts[0] == _verts[1] || 
    _verts[1] == _verts[2])
    return DegType::NONE;
  Geo::Point pt[3];
  double tol_sq = 0;
  for (auto i : { 0, 1, 2 })
  {
    _verts[i]->geom(pt[i]);
    tol_sq = std::max(tol_sq, Geo::epsilon_sq(pt[i]));
  }
  auto vec0 = pt[0] - pt[1];
  auto vec1 = pt[2] - pt[1];
  if (vec0 * vec1 < 0)
    return DegType::NONE;
  auto len0_sq = Geo::length_square(vec0);
  auto len1_sq = Geo::length_square(vec1);
  _dist_sq = Geo::length_square(vec0 % vec1) / std::max(len0_sq, len1_sq);
  if (_dist_sq > SAFE_TOL_MULTIPLE_SQ * tol_sq)
    return DegType::NONE;
  auto angle_threshold = 1.1 * Geo::angle(vec0, vec1);
  Topo::Iterator<Topo::Type::VERTEX, Topo::Type::EDGE> ve_it(_verts[1]);
  for (auto ed : ve_it)
  {
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev_it(ed);
    for (auto v : ev_it)
    {
      if (v == _verts[0] || v == _verts[1] || v == _verts[2])
        continue;
      Geo::Point ptv;
      v->geom(ptv);
      auto vec_oth = ptv - pt[1];
      if (Geo::angle(vec_oth, vec0) < angle_threshold ||
          Geo::angle(vec_oth, vec1) < angle_threshold)
      {
        return DegType::NONE;
      }
    }
  }
  if (Geo::length_square(pt[0] - pt[1]) < SAFE_TOL_MULTIPLE_SQ * tol_sq)
    return DegType::MERGE;
#if 0
  return DegType::NONE;
#else
  return len0_sq > len1_sq ? DegType::SPLITA : DegType::SPLITB;
#endif
}

void remove_degenerate_triangles(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::LOOP>& it_loop)
{
  for (auto loop : it_loop)
  {
    for (bool achange = true; achange; )
    {
      Topo::Iterator<Topo::Type::LOOP, Topo::Type::COEDGE> lc_it(loop);
      if (lc_it.size() < 3)
        break;
      achange = false;
      auto& last_coed = *std::prev(lc_it.end());
      Topo::Iterator<Topo::Type::COEDGE, Topo::Type::VERTEX> cv_it(last_coed);
      std::array<Topo::Wrap<Topo::Type::VERTEX>, 3> verts, deg_verts;
      verts[0] = *(cv_it.begin());
      verts[1] = *(std::next(cv_it.begin()));
      auto com_faces = 
        Topo::shared_entities<Topo::Type::VERTEX, Topo::Type::FACE>(verts[0], verts[1]);
      bool prev_inters_edge = com_faces.size() == 4;
      double min_dist_sq = std::numeric_limits<double>::max();
      DegType operation = DegType::NONE;
      for (auto coed : lc_it)
      {
        cv_it.reset(coed);
        verts[2] = *(std::next(cv_it.begin()));
        com_faces =
          Topo::shared_entities<Topo::Type::VERTEX, Topo::Type::FACE>(verts[1], verts[2]);
        bool inters_edge = com_faces.size() == 4;
        if (!inters_edge && !prev_inters_edge)
        {
          double dist_sq = 0;
          auto res = degenerate_triangle(verts.data(), dist_sq);
          static bool skip = false;
          if (!skip && res != DegType::NONE && dist_sq < min_dist_sq)
          {
            min_dist_sq = dist_sq;
            deg_verts = verts;
            operation = res;
          }
        }
        prev_inters_edge = inters_edge;
        std::copy(std::next(verts.begin()), verts.end(), verts.begin());
      }
      switch (operation)
      {
      case DegType::MERGE:
        achange |= Topo::merge(deg_verts[0], deg_verts[2]);
        break;
      case DegType::SPLITA:
        achange |= Topo::split(deg_verts[0], deg_verts[1], deg_verts[2]);
        break;
      case DegType::SPLITB:
        achange |= Topo::split(deg_verts[1], deg_verts[2], deg_verts[0]);
        break;
      }
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

struct DegenerateTriangleCLean
{
  void find(Topo::Wrap<Topo::Type::BODY>& _body);
  bool compute();
  struct DegSplit
  {
    double sq_dist_ = 0;
    Topo::Wrap<Topo::Type::VERTEX> verts_[3];
    DegType op_ = DegType::NONE;
    bool operator<(const DegSplit& _oth) { return sq_dist_ < _oth.sq_dist_; }
    bool fix()
    {
      if (op_ == DegType::MERGE)
        return Topo::merge(verts_[0], verts_[2]);
      else if (op_ == DegType::SPLITA)
        return Topo::split(verts_[0], verts_[1], verts_[2]);
      else if (op_ == DegType::SPLITB)
        return Topo::split(verts_[1], verts_[2], verts_[0]);
      return false;
    }
  };
  std::vector<DegSplit> split_datas_;
};

void DegenerateTriangleCLean::find(Topo::Wrap<Topo::Type::BODY>& _body)
{
#if 1
  Topo::Iterator<Topo::Type::BODY, Topo::Type::LOOP> it_loops(_body);
  for (auto loop : it_loops)
  {
    Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> it_verts(loop);
    if (it_verts.size() < 3)
      continue;
    DegSplit deg_split;
    deg_split.verts_[2] = it_verts.get(it_verts.size() - 2);
    deg_split.verts_[1] = it_verts.get(it_verts.size() - 1);
    for (auto v : it_verts)
    {
      deg_split.verts_[0] = v;
      deg_split.sq_dist_ = 0;
      deg_split.op_ = degenerate_triangle(deg_split.verts_, deg_split.sq_dist_);
      if (deg_split.op_ != DegType::NONE)
        split_datas_.push_back(deg_split);
      deg_split.verts_[2] = deg_split.verts_[1];
      deg_split.verts_[1] = deg_split.verts_[0];
    }
  }
#else
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> it_edges(_body);
  for (auto& edge : it_edges)
  {
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_verts(edge);
    if (it_verts.size() != 2)
      continue;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> it_faces(edge);

    for (size_t i = 0; i < 2; ++i)
    {
      DegSplit deg_split;
      deg_split.verts_[0] = it_verts.get(i);
      deg_split.verts_[1] = it_verts.get(1 - i);
      Topo::Iterator<Topo::Type::VERTEX, Topo::Type::EDGE> it_edges2(deg_split.verts_[1]);
      for (auto& edge2 : it_edges2)
      {
        if (edge2 == edge)
          continue;
        auto check_same_face = [&it_faces, &edge2]()
        {
          Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> it_faces2(edge2);
          for (auto f : it_faces)
            for (auto f2 : it_faces2)
              if (f == f2)
                return true;
          return false;
        };
        if (!check_same_face())
          continue;
        Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_verts2(edge2);
        for (auto& verts2 : it_verts2)
        {
          if (verts2 == deg_split.verts_[1])
            continue;
          deg_split.verts_[2] = verts2;
          deg_split.op_ = degenerate_triangle(deg_split.verts_, deg_split.sq_dist_);
          if (deg_split.op_ != DegType::NONE)
            split_datas_.push_back(deg_split);
        }
      }
    }
  }
#endif
}

bool DegenerateTriangleCLean::compute()
{
  std::sort(split_datas_.begin(), split_datas_.end());
  std::set < Topo::Wrap<Topo::Type::VERTEX>> used;
  bool achange = false;
  for (auto split_data : split_datas_)
  {
    for (auto v : split_data.verts_)
      if (used.find(v) != used.end())
        return achange;
    if (split_data.fix())
    {
      achange = true;
      for (auto v : split_data.verts_)
        used.insert(v);
    }
  }
  return achange;
}

}//namespace

 // Removes coincident consecutive vertices in face loops.
 // Removes faces with less than 3 vertices.
bool remove_degeneracies(
  Topo::Wrap<Topo::Type::BODY>& _body_1,
  Topo::Wrap<Topo::Type::BODY>& _body_2)
{
  Topo::Wrap<Topo::Type::BODY> bodies[2] = { _body_1 ,_body_2 };
  for (auto& body : bodies)
    remove_degenerate_edges(body);

  bool achange = false;
  size_t iter = 30;
  for(; iter > 0; --iter)
  {
    DegenerateTriangleCLean dtc;
    dtc.find(_body_1);
    dtc.find(_body_2);
    if (!dtc.compute())
      break;
    achange = true;
  }
  if (iter == 0)
  {
    std::cout << "Bad error non ending remove degeneracies.\n";
    achange = false;
  }

  for (auto& body : bodies)
  {
    Topo::Iterator<Topo::Type::BODY, Topo::Type::LOOP> it_loop(body);
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
  }
  return achange;
}

} // namespace Boolean