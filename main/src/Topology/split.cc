//#pragma optimize ("", off)
#include "geom.hh"
#include "impl.hh"
#include "shared.hh"
#include "split.hh"
#include "Geo/plane_fitting.hh"
#include "Geo/point_in_polygon.hh"
#include "Utils/circular.hh"
#include "Utils/continement_tree.hh"
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

void Split<Type::FACE>::use_face_loops(const LoopFilter& _loop_filter)
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
  if (_loop_filter(*loop_it))
  {
    make_chain(*loop_it, vert_chain);
    add_boundary(vert_chain);
  }
  while (++loop_it != fl_it.end())
  {
    if (_loop_filter(*loop_it))
    {
      make_chain(*loop_it, vert_chain);
      add_original_island(vert_chain);
    }
  }
}

namespace {

std::vector<Geo::VectorD3> vertex_chain_to_poly(VertexChain& _chain)
{
  std::vector<Geo::VectorD3> poly;
  poly.reserve(_chain.size());
  for (auto& v : _chain)
  {
    Geo::VectorD3 pt;
    v->geom(pt);
    poly.push_back(pt);
  }
  return poly;
}

bool is_inside(const std::vector<Geo::VectorD3>& _chain, 
  VertexChain& _isle, const Geo::VectorD3& _norm)
{
  Geo::VectorD3 pt_inside;
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
    auto select_chains = [&cur_islands, &poly, &norm](
      std::vector<VertexChain>& _island_chains)
    {
      for (auto& isle : _island_chains)
      {
        if (isle.empty())
          continue;
        if (is_inside(poly, isle, norm))
          cur_islands.emplace_back(std::move(isle));
      }
    };
    select_chains(original_island_chains_);
    auto original_size = cur_islands.size();
    select_chains(island_chains_);
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
      if (original_size > 0)
      {
        --original_size;
        continue;
      }
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

namespace
{
void loop_split(Wrap<Type::FACE>& _face, 
                Iterator<Type::FACE, Type::LOOP>& _loop_it,
                const Wrap<Type::VERTEX>& _ins_vert)
{
  auto loop = *(_loop_it.begin());
  std::vector<size_t> ins_pos;
  for (auto pos = SIZE_MAX;;)
  {
    pos = loop->find_child(_ins_vert.get(), pos);
    if (pos == SIZE_MAX)
      break;
    ins_pos.push_back(pos);
  }
  if (ins_pos.size() < 2)
    return;
  std::vector<VertexChain> chains;
  size_t last_ind = loop->size(Direction::Down) - 1;
  for (auto pos : ins_pos)
  {
    chains.emplace_back();
    for (size_t i = pos; i <= last_ind; ++i)
    {
      auto v = loop->get(Direction::Down, i);
      chains.back().emplace_back(static_cast<E<Type::VERTEX> *>(v));
    }
    last_ind = pos;
  }
  if (ins_pos.back() == 0)
    chains.front().push_back(_ins_vert);
  else
  {
    auto& extend_chain = chains.front();
    for (size_t i = 0; i <= ins_pos.back(); ++i)
    {
      auto v = loop->get(Direction::Down, i);
      extend_chain.emplace_back(static_cast<E<Type::VERTEX> *>(v));
    }
  }
  auto compare_chains = [](const VertexChain* _a, const VertexChain* _b)
  {
    int result = 0;
    auto inside = [&result](const VertexChain* _a, const VertexChain* _b, int _out)
    {
      Geo::Point pt;
      (*_a)[1]->geom(pt);
      if (PointInFace::classify((*_b), pt) != Geo::PointInPolygon::Inside)
        return false;
      result = _out;
      return true;
    };
    inside(_a, _b, -1) || inside(_b, _a, 1);
    return result;
  };
  Utils::ContainementTree<VertexChain*> cont_tree(compare_chains);
  for (auto& ch : chains)
    cont_tree.add(&ch);
  auto root = cont_tree.root();
  if (!root->next())
    return;
  Split<Type::FACE> face_splitter(_face);
  struct SelectIsles : public Split<Type::FACE>::LoopFilter
  {
    SelectIsles(const Wrap<Type::LOOP>& _loop) : loop_(_loop) {}
    bool operator()(const Topo::Wrap<Topo::Type::LOOP>& _loop) const
    {
      return loop_ != _loop;
    }
    Wrap<Type::LOOP> loop_;
  } sel_isles(loop);
  face_splitter.use_face_loops(sel_isles);
  for (; root != nullptr; root = root->next())
  {
    auto vc = root->data();
    for (auto conn = root->child(); conn != nullptr; conn = conn->next())
      vc->insert(vc->end(), conn->data()->begin(), conn->data()->end());
    face_splitter.add_boundary(*vc);
  }
  face_splitter.compute();
}

}

bool split(const Wrap<Type::VERTEX>& _ed_start,
  const Wrap<Type::VERTEX>& _ed_end, Wrap<Type::VERTEX>& _ins_vert)
{
  auto loops = shared_entities<Type::VERTEX, Type::LOOP>(_ed_start, _ed_end);
  for (auto& loop : loops)
  {
    for (auto pos = SIZE_MAX;;)
    {
      pos = loop->find_child(_ed_start.get(), pos);
      if (pos == SIZE_MAX)
        break;
      auto oth = Utils::increase(pos, loop->size(Direction::Down));
      if (loop->get(Direction::Down, oth) == _ed_end.get())
        loop->insert_child(_ins_vert.get(), pos + 1);
      oth = Utils::decrease(pos, loop->size(Direction::Down));
      if (loop->get(Direction::Down, oth) == _ed_end.get())
        loop->insert_child(_ins_vert.get(), pos);
    }
    // CHeck that the loop is aface boundary
    Iterator<Type::LOOP, Type::FACE> faces(loop);
    if (faces.size() != 1)
      continue;
    auto face = *faces.begin();
    Iterator<Type::FACE, Type::LOOP> face_loops(face);
    if (*face_loops.begin() != loop)
      continue;
    loop_split(face, face_loops, _ins_vert);
  }
  return true;
}

}
