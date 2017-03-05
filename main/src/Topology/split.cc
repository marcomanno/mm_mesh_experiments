#pragma optimize ("", off)
#include "impl.hh"
#include "shared.hh"
#include "split.hh"
#include "Utils/circular.hh"
#include "Utils/error_handling.hh"

#include <algorithm>

namespace Topo {

void Split<Type::EDGE>::add_point(const Info& inf_) const
{
  split_pts_.push_back(inf_);
}

void Split<Type::EDGE>::remove_duplicates()
{
  auto new_end = std::unique(split_pts_.begin(), split_pts_.end(), 
    [](const Info& _a, const Info& _b) { return _a.vert_ == _b.vert_; });
  split_pts_.erase(new_end, split_pts_.end());
  if (split_pts_.size() > 1 && split_pts_.front().vert_ == split_pts_.back().vert_)
    split_pts_.pop_back();
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
      //THROW("Impossible splt edge");
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
  face_->remove();
  return true;
}

bool split(const Wrap<Type::VERTEX>& _ed_start,
  const Wrap<Type::VERTEX>& _ed_end, Wrap<Type::VERTEX>& _ins_vert)
{
  auto faces = shared_entities<Type::VERTEX, Type::FACE>(_ed_start, _ed_end);
  for (auto& f : faces)
  {
    for (auto pos = SIZE_MAX;;)
    {
      pos = f->find_child(_ed_start.get(), pos);
      if (pos == SIZE_MAX)
        break;
      auto oth = Utils::increase(pos, f->size(Direction::Down));
      if (f->get(Direction::Down, oth) == _ed_end.get())
        f->insert_child(_ins_vert.get(), pos + 1);
      oth = Utils::decrease(pos, f->size(Direction::Down));
      if (f->get(Direction::Down, oth) == _ed_end.get())
        f->insert_child(_ins_vert.get(), pos);
    }
  }
  return true;
}

}
