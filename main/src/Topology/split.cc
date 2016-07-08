#include "impl.hh"
#include "split.hh"
#include "Utils/error_handling.hh"

#include <algorithm>

namespace Topo {

void Split<Type::EDGE>::add_point(const Info& inf_) const
{
  split_pts_.push_back(inf_);
}

bool Split<Type::EDGE>::operator()() const
{
  std::sort(split_pts_.begin(), split_pts_.end(), 
    [](const Info&_a, const Info&_b) { return _a.t_ < _b.t_; });
  if (edge_->sub_type() != SubType::EDGE_REF)
    return false;
  auto ed_ref = static_cast<const EdgeRef*>(edge_.get());
  std::vector<IBase*> faces[2];
  const EE<Type::VERTEX>* vert_impl[2];
  for (size_t i = 0; i < std::size(ed_ref->verts_); ++i)
  {
    vert_impl[i] = static_cast<const EE<Type::VERTEX>*>(ed_ref->verts_[i].get());
    size_t face_nmbr = vert_impl[i]->size(Direction::Up);
    for (size_t j = 0; j < face_nmbr; ++j)
      faces[i].push_back(vert_impl[i]->get(Direction::Up, j));
    std::sort(faces[i].begin(), faces[i].end());
  }
  std::vector<IBase*> comm_faces;
  std::set_intersection(
    faces[0].begin(), faces[0].end(),
    faces[1].begin(), faces[1].end(),
    std::back_inserter(comm_faces));
  for (auto face : comm_faces)
  {
    size_t pos = SIZE_MAX;
    for (;;)
    {
      pos = face->find_child(vert_impl[0], pos);
      if (pos == SIZE_MAX)
        break;
      auto pos1 = (pos + 1) % face->size(Direction::Down);
      if (face->get(Direction::Down, pos1) == vert_impl[1])
      {
        for (auto it = split_pts_.rbegin(); it != split_pts_.rend(); ++it)
          face->insert_child(it->vert_.get(), pos1);
        continue;
      }
      pos1 = pos == 0 ? face->size(Direction::Down) - 1 : pos - 1;
      if (face->get(Direction::Down, pos1) == vert_impl[1])
      {
        for (auto it = split_pts_.begin(); it != split_pts_.end(); ++it)
          face->insert_child(it->vert_.get(), pos);
        continue;
      }
      THROW("Impossible splt edge");
    }
  }

  return true;
}


bool Split<Type::FACE>::operator()(VertexChains& _chains)
{
  if (_chains.empty())
    return false;
  for (auto& chain : _chains)
  {
    Wrap<Type::FACE> new_face;
    new_face.make<EE<Type::FACE>>();
    for (auto& vert : chain)
      new_face->insert_child(vert.get());
    
    for (size_t i = 0; i < face_->size(Direction::Up); ++i)
    {
      auto parent = face_->get(Direction::Up, i);
      parent->insert_child(new_face.get());
    }
    new_faces_.emplace_back(new_face);
  }
  for (size_t i = face_->size(Direction::Down); i-- > 0;)
    face_->remove_child(i);
  face_->remove();
  return true;
}

}
