
#pragma optimize ("", off)
#include "priv.hh"

#include "Topology/impl.hh"
#include "Topology/merge.hh"
#include "Topology/shared.hh"



namespace Boolean {

// Removes coincident consecutive vertices in face loops.
// Removes faces with less than 3 vertices.
bool remove_degeneracies(Topo::Wrap<Topo::Type::BODY>& _body)
{
  for (bool achange = true; achange; )
  {
    achange = false;
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> it_ed(_body);
    for (auto edge : it_ed)
    {
      Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> vv(edge);
      if (vv.get(0) == vv.get(1))
        continue;
      Geo::Segment seg;
      edge->geom(seg);
      auto tol = edge->tolerance();
      if (Geo::length_square(seg[1] - seg[0]) > 3 * Geo::sq(tol))
        continue;
      Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> it_ve(edge);
      achange |= Topo::merge(it_ve.get(0), it_ve.get(1));
    }
  }
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> it_fa(_body);
  bool achange = false;
  for (auto face : it_fa)
  {
    if (!face.get())
      continue;
    bool loc_change;
    do {
      loc_change = false;
      auto n_verts = face->size(Topo::Direction::Down);
      if (n_verts >= 3)
      {
        auto prev_vert_ptr = face->get(Topo::Direction::Down, 0);
        for (size_t j = n_verts; j-- > 0; )
        {
          auto curr_vert_ptr = face->get(Topo::Direction::Down, j);
          if (curr_vert_ptr == prev_vert_ptr)
          {
            loc_change = true;
            face->remove_child(j); // curr_vert_ptr->remove();
          }
          else
            prev_vert_ptr = curr_vert_ptr;
        }
      }
      Topo::Iterator<Topo::Type::FACE, Topo::Type::COEDGE> fe_it(face);
      if (fe_it.size() >= 4)
      {
        Topo::Iterator<Topo::Type::COEDGE, Topo::Type::EDGE> 
          prev_ed_it(*std::prev(fe_it.end()));
        auto prev_ed = *prev_ed_it.begin();
        std::vector<Topo::Wrap<Topo::Type::VERTEX>> rem_verts;
        for (auto ced : fe_it)
        {
          Topo::Iterator<Topo::Type::COEDGE, Topo::Type::EDGE> ed_it(ced);
          auto ed = *ed_it.begin();
          if (ed == prev_ed)
          {
            auto same_body_faces = [](Topo::Wrap<Topo::Type::EDGE> _ed)
            {
              Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> ef_it(_ed);
              auto it = ef_it.begin();
              auto b = (*it)->get(Topo::Direction::Up, 0);
              while (++it != ef_it.end())
                if (b != (*it)->get(Topo::Direction::Up, 0))
                  return false;
              return true;
            };
            if (same_body_faces(ed))
            {
              Topo::Iterator<Topo::Type::COEDGE, Topo::Type::VERTEX> cv_it(ced);
              rem_verts.push_back(cv_it.get(0));
            }
          }
          prev_ed = ed;
        }
        for (auto v : rem_verts)
          face->remove_child(v.get());
        loc_change |= !rem_verts.empty();
      }
      achange |= loc_change;
    } while (loc_change);
    if (face->size(Topo::Direction::Down) < 3)
    {
      achange |= true;
      face->remove();
    }
  }
  return achange;
}

}