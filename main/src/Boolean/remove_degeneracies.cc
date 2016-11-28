
#pragma optimize ("", off)
#include "priv.hh"

#include "Topology/impl.hh"

namespace Boolean {

// Removes coincident consecutive vertices in face loops.
// Removes faces with less than 3 vertices.
bool remove_degeneracies(Topo::Wrap<Topo::Type::BODY>& _body)
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> it;
  it.reset(_body);
  bool achange = false;
  for (size_t i = 0; i < it.size(); ++i)
  {
    auto face = it.get(i);
    if (!face.get())
      continue;
    bool loc_change;
    do {
      loc_change = false;
      auto n_verts = face->size(Topo::Direction::Down);
      if (n_verts >= 4)
      {
        size_t j_prev = 0;
        for (size_t j = n_verts; j-- > 0; j_prev = j)
        {
          size_t j_next = j > 0 ?
            j - 1 : face->size(Topo::Direction::Down);
          if (face->get(Topo::Direction::Down, j_prev) ==
            face->get(Topo::Direction::Down, j_next))
          {
            loc_change = true;
            face->remove_child(j);//  get(Topo::Direction::Down, j)->remove();
          }
        }
      }
      n_verts = face->size(Topo::Direction::Down);
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