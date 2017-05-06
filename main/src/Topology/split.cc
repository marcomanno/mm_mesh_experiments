#pragma optimize ("", off)
#include "impl.hh"
#include "shared.hh"
#include "split.hh"
#include "Geo/plane_fitting.hh"
#include "Geo/point_in_polygon.hh"
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

void Split<Type::FACE>::use_face_loops()
{
  Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> fl_it(face_);
  if (fl_it.size() == 0)
    return;
  auto make_chain = [](const Topo::Wrap<Topo::Type::LOOP>& _loop,
    VertexChain& _vert_chain)
  {
    Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> lv_it(_loop);
    for (auto& v : lv_it)
      _vert_chain.emplace_back(v);
  };
  auto loop_it = fl_it.begin();
  VertexChain vert_chain;
  make_chain(*loop_it, vert_chain);
  add_boundary(vert_chain);
  while (++loop_it != fl_it.end())
  {
    make_chain(*loop_it, vert_chain);
    add_island(vert_chain);
  }
}

namespace {

std::vector<Geo::Vector3> vertex_chain_to_poly(VertexChain& _chain)
{
  std::vector<Geo::Vector3> poly;
  poly.reserve(_chain.size());
  for (auto& v : _chain)
  {
    Geo::Vector3 pt;
    v->geom(pt);
    poly.push_back(pt);
  }
  return poly;
}

bool is_inside(const std::vector<Geo::Vector3>& _chain, 
  VertexChain& _isle, const Geo::Vector3& _norm)
{
  Geo::Vector3 pt_inside;
  _isle[0]->geom(pt_inside);
  auto pt_clss = Geo::PointInPolygon::classify(_chain, pt_inside, &_norm);
  if (pt_clss != Geo::PointInPolygon::Inside)
    return false;
  auto int_norm = Geo::vertex_polygon_normal(_isle.begin(), _isle.end());
  if (int_norm * _norm > 0)
    std::reverse(_isle.begin(), _isle.end());
  return true;
}

} // namespace

bool Split<Type::FACE>::compute()
{
  if (boundary_chains_.empty())
    return false;
  for (auto& chain : boundary_chains_)
  {
    if (chain.empty())
      continue;
    auto poly = vertex_chain_to_poly(chain);
    std::vector<VertexChain> cur_islands;
    auto norm = Geo::vertex_polygon_normal(chain.begin(), chain.end());
    for (auto& isle : island_chains_)
    {
      if (isle.empty())
        continue;
      if (is_inside(poly, isle, norm))
        cur_islands.emplace_back(std::move(isle));
    }
    bool make_loops = !cur_islands.empty();

	  auto inherit_parents = [this](Wrap<Type::FACE>& _new_face)
    {
        for (size_t i = 0; i < face_->size(Direction::Up); ++i)
        {
          auto parent = face_->get(Direction::Up, i);
          parent->insert_child(_new_face.get());
        }		
    };
	  auto add_face_loop = [](
	    Wrap<Type::FACE> _new_face, VertexChain& _chain, bool _make_loops)
    {
      IBase* up_elem = _new_face.get();
      if (_make_loops)
      {
        Wrap<Type::LOOP> new_loop;
        new_loop.make<EE<Type::LOOP>>();
        _new_face->insert_child(new_loop.get());
        up_elem = new_loop.get();
      }
      else
        up_elem = _new_face.get();
      for (auto& vert : _chain)
        up_elem->insert_child(vert.get());
    };
    Wrap<Type::FACE> new_face;
    new_face.make<EE<Type::FACE>>();
    add_face_loop(new_face, chain, make_loops);
    for (auto loop : cur_islands)
    {
      add_face_loop(new_face, loop, make_loops);
      Wrap<Type::FACE> new_int_face;
      new_int_face.make<EE<Type::FACE>>();
      std::reverse(loop.begin(), loop.end());
      add_face_loop(new_int_face, loop, false);
      inherit_parents(new_int_face);
      new_faces_.emplace_back(new_int_face);
    }
    inherit_parents(new_face);
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
