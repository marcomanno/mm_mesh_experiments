
#include "priv.hh"

#include "Topology/impl.hh"

namespace Boolean {

bool remove_degeneracies(Topo::Wrap<Topo::Type::BODY>& _body)
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> it;
  it.reset(_body);
  bool chngd = false;
  for (size_t i = 0; i < it.size(); ++i)
  {
    auto face = it.get(i);
    auto face_ptr = dynamic_cast<Topo::UpEntity<Topo::Type::FACE>*>(face.get());
    if (!face_ptr)
      continue;
    auto n_verts = face_ptr->size(Topo::Direction::Down);
    if (n_verts >= 3)
    {
      auto prev_vert_ptr = face_ptr->get(Topo::Direction::Down, 0);
      for (size_t j = n_verts; j-- > 0; )
      {
        auto curr_vert_ptr = face_ptr->get(Topo::Direction::Down, j);
        if (curr_vert_ptr == prev_vert_ptr)
        {
          chngd |= true;
          curr_vert_ptr->remove();
        }
        else
          prev_vert_ptr = curr_vert_ptr;
      }
    }
    if (face_ptr->size(Topo::Direction::Down) < 3)
    {
      chngd |= true;
      face_ptr->remove();
    }
  }
  return chngd;
}

}