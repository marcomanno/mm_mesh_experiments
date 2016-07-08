
#include "priv.hh"
#include "Utils/statistics.hh"
#include "Topology/iterator.hh"
#include "Topology/impl.hh"
#include "Topology/split.hh"
#include "Geo/entity.hh"
#include "Geo/vector.hh"
#include "Geo/minsphere.hh"
#include "Utils/index.hh"
#include "Utils/merger.hh"

#include "Utils/error_handling.hh"

#include <set>
#include <limits>

namespace Boolean {

namespace {

struct EdgeEdgeSplintInfo : public Utils::Mergiable
{
  std::shared_ptr<Topo::Wrap<Topo::Type::VERTEX>> vert_;
  Geo::Point pt_;
  double tol_ = 0;
  struct EdgeSplintInfo
  {
    bool on_end = false;
    Topo::Wrap<Topo::Type::EDGE> edge_;
    double par = 0;
  } ed_split_info_[2];
};

void merge_intersections(std::vector<EdgeEdgeSplintInfo>& _splt_inf)
{
  for (size_t i = 0; i < _splt_inf.size(); ++i)
  {
    for (size_t j = 0; j < _splt_inf.size(); ++j)
    {
      Utils::FindMax<double> max_tol({ _splt_inf[i].tol_, _splt_inf[j].tol_ });
      if (!Geo::same(_splt_inf[i].pt_, _splt_inf[j].pt_, max_tol()))
        return;
      THROW_IF(_splt_inf[i].vert_ && _splt_inf[j].vert_,
        "This is not managed. It is necessary to merge 2 existing vertices!");
      EdgeEdgeSplintInfo* eei[2] = { &_splt_inf[i], &_splt_inf[j] };
      if (!_splt_inf[i].vert_ && !_splt_inf[j].vert_)
      {
        if (_splt_inf[j].equiv_idx_ == Utils::INVALID_INDEX)
          _splt_inf[j].equiv_idx_ = i;
        else if (_splt_inf[i].equiv_idx_ == Utils::INVALID_INDEX)
          _splt_inf[j].equiv_idx_ = j;
        else
        {
          for (size_t k = j, k_next; _splt_inf[k].equiv_idx_ != Utils::INVALID_INDEX; k = k_next)
          {
            k_next = _splt_inf[k].equiv_idx_;
            _splt_inf[k].equiv_idx_ = _splt_inf[i].equiv_idx_;
          }
        }
      }
      else
      {
        if (!_splt_inf[i].vert_)
          std::swap(i, j);
        _splt_inf[j].vert_ = _splt_inf[i].vert_;
        _splt_inf[j].tol_ = _splt_inf[i].tol_ = max_tol();
        _splt_inf[j].pt_ = _splt_inf[i].pt_;
      }
    }
  }
  std::vector<std::tuple<std::vector<Geo::Point>, double>> ball_pts(_splt_inf.size());
  for (size_t i = 0; i < _splt_inf.size(); ++i)
  {
    if (_splt_inf[i].equiv_idx_ == Utils::INVALID_INDEX)
      continue;
    size_t k = i;
    for (; _splt_inf[k].equiv_idx_ != Utils::INVALID_INDEX; k = _splt_inf[k].equiv_idx_);
    _splt_inf[i].equiv_idx_ = k;
    if (!_splt_inf[k].vert_)
    {
      _splt_inf[k].vert_ = std::make_shared<Topo::Wrap<Topo::Type::VERTEX>>();
      _splt_inf[k].vert_->make<Topo::EE<Topo::Type::VERTEX>>();
      std::get<0>(ball_pts[k]).push_back(_splt_inf[k].pt_);
      std::get<double>(ball_pts[k]) = _splt_inf[k].tol_;
    }
    _splt_inf[i].vert_ = _splt_inf[k].vert_;
    std::get<0>(ball_pts[k]).push_back(_splt_inf[i].pt_);
    std::get<double>(ball_pts[k]) = std::max(std::get<double>(ball_pts[k]), _splt_inf[i].tol_);
  }
  for (size_t i = 0; i < _splt_inf.size(); ++i)
  {
    const auto& pts = std::get<0>(ball_pts[i]);
    if (pts.empty())
      continue;
    auto best_sphere = Geo::min_ball(pts.data(), pts.size());
    double tol = best_sphere.radius_ + std::get<double>(ball_pts[i]);
    (*_splt_inf[i].vert_)->set_geom(best_sphere.centre_);
    (*_splt_inf[i].vert_)->set_tolerance(tol);
  }
}

struct EdgeVersusEdges : public IEdgeVersusEdges
{
  virtual bool intersect(
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_a,
    Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_b);

  virtual bool split();
private:
  std::vector<EdgeEdgeSplintInfo> splt_infos_;
};

bool EdgeVersusEdges::intersect(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_a,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE>& _ed_it_b)
{
  struct IntersectionData
  {
    Topo::Wrap<Topo::Type::EDGE> edge_;
    Geo::Segment seg_;
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev_it_;
    double tol_ = 0;

    void set_edge(const Topo::Wrap<Topo::Type::EDGE>& _edge)
    {
      edge_ = _edge;
      edge_->geom(seg_);
      ev_it_.reset(edge_);
      tol_ = edge_->tolerance();
    }

    bool inersection_on_end(const Geo::Point& _clsst_pt, const double _tol,
      EdgeEdgeSplintInfo::EdgeSplintInfo& _splt_info, 
      std::shared_ptr<Topo::Wrap<Topo::Type::VERTEX>>& _vert)
    {
      Geo::Point vert_pt;
      for (size_t j = 0; j < ev_it_.size(); ++j)
      {
        ev_it_.get(j)->geom(vert_pt);
        if (!Geo::same(vert_pt, _clsst_pt, _tol))
          continue;
        _splt_info.on_end = true;
        _vert = std::make_shared<Topo::Wrap<Topo::Type::VERTEX>>(ev_it_.get(j));
        return true;
      }
      return false;
    }

  } intrs_dat[2];

  for (size_t i = 0; i < _ed_it_a.size(); ++i)
  {
    intrs_dat[0].set_edge(_ed_it_a.get(i));
    for (size_t j = 0; j < _ed_it_b.size(); ++j)
    {
      intrs_dat[1].set_edge(_ed_it_b.get(j));

      std::vector<std::array<size_t, 2>> matches;
      for (size_t k = 0; k < intrs_dat[0].ev_it_.size(); ++k)
      {
        for (size_t l = 0; l < intrs_dat[1].ev_it_.size(); ++l)
        {
          if (intrs_dat[0].ev_it_.get(k) == intrs_dat[1].ev_it_.get(l))
          {
            matches.push_back({ k, l });
            break;
          }
        }
      }
      if (matches.size() == 2)
        continue;
      Geo::Point clsst_pt;
      double pars[2], dist;
      if (!Geo::closest_point(
        intrs_dat[0].seg_, intrs_dat[1].seg_,
        &clsst_pt, pars, &dist))
      {
        continue;
      }
      Utils::FindMax<double> max_tol(intrs_dat[0].tol_);
      max_tol.add(intrs_dat[1].tol_);
      if (dist > max_tol())
        continue;

      EdgeEdgeSplintInfo ed_ed_splt_inf;
      size_t on_end_nmbr = 0;
      ed_ed_splt_inf.pt_ = clsst_pt;
      for (size_t k = 0; k < 2; ++k)
      {
        bool on_end = intrs_dat[k].inersection_on_end(
          clsst_pt, max_tol(), 
          ed_ed_splt_inf.ed_split_info_[k], ed_ed_splt_inf.vert_);
        if (on_end)
        {
          (*ed_ed_splt_inf.vert_)->geom(ed_ed_splt_inf.pt_);
          ++on_end_nmbr;
        }
      }
      if (on_end_nmbr == 2)
        continue; // Nothing to split, intersection is on end of both edges.

      for (size_t k = 0; k < 2; ++k)
      {
        ed_ed_splt_inf.ed_split_info_[k].edge_ = intrs_dat[k].edge_;
        ed_ed_splt_inf.ed_split_info_[k].par = pars[k];
      }
      ed_ed_splt_inf.tol_ = max_tol();
      splt_infos_.push_back(ed_ed_splt_inf);
    }
  }
  return true;
}

bool EdgeVersusEdges::split()
{
  merge_intersections(splt_infos_);

  std::set<Topo::Split<Topo::Type::EDGE>> ed_splt_set;
  for (auto& splt_info : splt_infos_)
  {
    if (!splt_info.vert_)
    {
      splt_info.vert_ = std::make_shared<Topo::Wrap<Topo::Type::VERTEX>>();
      auto vert = splt_info.vert_->make<Topo::EE<Topo::Type::VERTEX>>();
      vert->set_geom(splt_info.pt_);
      vert->set_tolerance(splt_info.tol_);
    }

    auto skip_vert_on_edge_check =
      !splt_info.ed_split_info_[0].on_end && !splt_info.ed_split_info_[1].on_end;
    for (auto& ed_split : splt_info.ed_split_info_)
    {
      if (ed_split.on_end)
        continue;
      if (!skip_vert_on_edge_check)
      {
        Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev_it;
        bool vertex_already_on_edge = false;
        for (size_t i = 0; i < ev_it.size() && !vertex_already_on_edge; ++i)
          vertex_already_on_edge = (ev_it.get(i) == *splt_info.vert_);
        if (vertex_already_on_edge)
          continue;
      }

      auto it = ed_splt_set.lower_bound(ed_split.edge_);
      if (it == ed_splt_set.end() || *it != ed_split.edge_)
        it = ed_splt_set.emplace_hint(it, ed_split.edge_);

      Topo::Split<Topo::Type::EDGE>::Info ed_split_info;
      ed_split_info.vert_ = *splt_info.vert_;
      ed_split_info.t_ = ed_split.par;
      ed_split_info.dist_ = splt_info.tol_;
      it->add_point(ed_split_info);
    }
  }
  for (auto split_op : ed_splt_set)
    split_op();
  return true;
}


}//namespace

std::shared_ptr<IEdgeVersusEdges> IEdgeVersusEdges::make()
{
  return std::make_shared<EdgeVersusEdges>();
}

}//namespace Boolean
