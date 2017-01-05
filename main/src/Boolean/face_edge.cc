#pragma optimize ("", off)
#include "face_intersections.hh"
#include "Geo/MinSphere.hh"
#include "Geo/kdtree.hh"
#include "Geo/vector.hh"
#include "Topology/split.hh"
#include "Topology/impl.hh"
#include "Utils/index.hh"
#include "Utils/merger.hh"

#include <set>

namespace Boolean {

namespace {

struct FaceEdgeInfo
{
  std::map<Topo::Wrap<Topo::Type::FACE>, std::vector<Utils::Index>> f_v_refs_;

  typedef double Parameter;
  typedef std::tuple<Utils::Index, Parameter> EdgeVertexReference;
  std::map<Topo::Wrap<Topo::Type::EDGE>, std::vector<EdgeVertexReference>> e_v_refs_;

  struct VertexReferences : public Utils::Mergiable
  {
    Utils::Index this_idx_;
    Geo::Point pt_;
    double tol_;
    Topo::Wrap<Topo::Type::VERTEX> vert_;
    std::vector<Geo::Point> mrg_list_;
    std::set<Topo::Wrap<Topo::Type::EDGE>> edge_refs_;
    std::set<Topo::Wrap<Topo::Type::FACE>> face_refs_;
    FaceEdgeInfo* owner_;

    bool equivalent(const VertexReferences& _oth) const;
    void merge(const VertexReferences& _oth);
  };
  std::vector<VertexReferences> vertices_refs_;

  void add(const Geo::Point& _pt, const double _t,
    const Topo::Wrap<Topo::Type::EDGE>& _edge,
    const Topo::Wrap<Topo::Type::FACE>& _face);

  void merge();

  void split_edges();
};

bool FaceEdgeInfo::VertexReferences::equivalent(const VertexReferences& _oth) const
{
  return Geo::same(pt_, _oth.pt_, std::max(tol_, _oth.tol_));
}

void FaceEdgeInfo::VertexReferences::merge(const VertexReferences& _oth)
{
  if (mrg_list_.empty())
    mrg_list_.emplace_back(pt_);
  mrg_list_.emplace_back(_oth.pt_);
  tol_ = std::max(tol_, _oth.tol_);
  for (const auto& face : _oth.face_refs_)
  {
    auto& f_v = owner_->f_v_refs_[face];
    std::replace(f_v.begin(), f_v.end(), _oth.this_idx_, _oth.equiv_idx_);
  }
  for (const auto& edge : _oth.edge_refs_)
  {
    auto& e_v = owner_->e_v_refs_[edge];
    for (auto vert_ref : e_v)
    {
      auto& idx = std::get<0>(vert_ref);
      if (idx == _oth.this_idx_)
        idx = _oth.equiv_idx_;
    }
  }
  edge_refs_.insert(_oth.edge_refs_.begin(), _oth.edge_refs_.end());
  face_refs_.insert(_oth.face_refs_.begin(), _oth.face_refs_.end());
}

void FaceEdgeInfo::add(
  const Geo::Point& _pt, const double _t,
  const Topo::Wrap<Topo::Type::EDGE>& _edge,
  const Topo::Wrap<Topo::Type::FACE>& _face)
{
  auto idx = vertices_refs_.size();
  f_v_refs_[_face].push_back(idx);
  e_v_refs_[_edge].emplace_back(idx, _t);
  vertices_refs_.emplace_back();
  auto& new_el = vertices_refs_.back();
  new_el.pt_ = _pt;
  new_el.tol_ = _edge->tolerance();
  new_el.edge_refs_.emplace(_edge);
  new_el.face_refs_.emplace(_face);
  new_el.owner_ = this;
  new_el.this_idx_ = idx;
}

void FaceEdgeInfo::merge()
{
  Utils::merge(vertices_refs_);
  for (auto& vert : vertices_refs_)
  {
    if (vert.equiv_idx_ != Utils::INVALID_INDEX)
      continue;
    if (!vert.mrg_list_.empty())
    {
      auto sphere = Geo::min_ball(vert.mrg_list_.data(), vert.mrg_list_.size());
      vert.pt_ = sphere.centre_;
      if (sphere.radius_ > vert.tol_)
        vert.tol_ = sphere.radius_;
    }
  }
}

void FaceEdgeInfo::split_edges()
{
  for (auto& edge_info : e_v_refs_)
  {
    auto edge = edge_info.first;
    Topo::Split<Topo::Type::EDGE> splitter(edge);
    for (auto split_idx : edge_info.second)
    {
      auto& split_vert = this->vertices_refs_[std::get<Utils::Index>(split_idx)];
      if (!split_vert.vert_)
      {
        split_vert.vert_.make<Topo::EE<Topo::Type::VERTEX>>();
        split_vert.vert_->set_geom(split_vert.pt_);
        split_vert.vert_->set_tolerance(split_vert.tol_);
      }
      Topo::Split<Topo::Type::EDGE>::Info splt_pt_info;
      splt_pt_info.vert_ = split_vert.vert_;
      splt_pt_info.t_ = std::get<Parameter>(split_idx);
      splt_pt_info.clsst_pt_ = split_vert.pt_;
      splt_pt_info.dist_ = split_vert.tol_;
      splitter.add_point(splt_pt_info);
    }
    splitter();
  }
}

}//namespace


bool FaceVersus::edge_intersect(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE>& _face_it,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _edge_it)
{
  Geo::KdTree<Topo::Wrap<Topo::Type::FACE>> kdfaces;
  kdfaces.insert(_face_it.begin(), _face_it.end());
  kdfaces.compute();
  Geo::KdTree<Topo::Wrap<Topo::Type::EDGE>> kdedges;
  kdedges.insert(_edge_it.begin(), _edge_it.end());
  kdedges.compute();
  auto pairs = Geo::find_kdtree_couples<
    Topo::Wrap<Topo::Type::FACE>,
    Topo::Wrap<Topo::Type::EDGE>>(kdfaces, kdedges);

#ifdef DEBUG_KDTREE
  auto old_pairs(std::move(pairs));
  for (size_t i = 0; i < _face_it.size(); ++i)
    for (size_t j = 0; j < _edge_it.size(); ++j)
      pairs.emplace_back(std::array<size_t, 2>{ i, j });
#endif

  FaceEdgeInfo f_eds_info;
  for (const auto& pair : pairs)
  {
    auto face = kdfaces[pair[0]];
    auto& face_info = face_geom(kdfaces[pair[0]]);
    auto edge = kdedges[pair[1]];
    Geo::Segment seg;
    edge->geom(seg);
    Geo::Point clsst_pt;
    double dist_sq, t_seg;
    if (!closest_point(*face_info.poly_face_, seg, &clsst_pt, &t_seg, &dist_sq))
      continue;
    auto pt_tol = Geo::epsilon(clsst_pt);
    if (dist_sq > std::max(pt_tol, edge->tolerance()))
      continue;

#ifdef DEBUG_KDTREE
    if (std::find(old_pairs.begin(), old_pairs.end(), pair) == old_pairs.end())
    {
      bool box_inters = (kdfaces[pair[0]]->box() * kdedges[pair[1]]->box()).empty();
      std::cout << "Error " << pair[0] << " " << pair[1] << " Box inters " << box_inters << std::endl;
    }
#endif

    bool point_on_vertex = false;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev_it(edge);
    for (auto vert : ev_it)
    {
      Geo::Point pt;
      vert->geom(pt);
      if (Geo::same(pt, clsst_pt, std::max(pt_tol, vert->tolerance())))
      {
        point_on_vertex = true;
        // Todo: verify that the vertex is at the edge end.
        break;
      }
    }
    if (point_on_vertex) // Intersection is at edge end.
      continue;

    f_eds_info.add(clsst_pt, t_seg, edge, face);
  }
  f_eds_info.merge(); // Merge the intersection points.
  f_eds_info.split_edges(); // Splits the edges.
  for (auto fv : f_eds_info.f_v_refs_) // Adds the vertices in the face vertex list.
  {
    const auto& face = fv.first;
    auto& face_vert_list = f_vert_info_[face];
    for (const auto& vert_info : fv.second)
    {
      face_vert_list.new_vert_list_.push_back(f_eds_info.vertices_refs_[vert_info].vert_);
    }
  }
  return true;
}

}//namespace Boolean
