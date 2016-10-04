
#include "priv.hh"
#include "Geo/vector.hh"
#include "Topology/geom.hh"
#include "Utils/error_handling.hh"

#include <set>
#include <list>

namespace Boolean {

namespace {

enum Choice { REMV, KEEP, INVR };

struct Selection : public ISelection
{
  Selection(Operation _bool_op) : bool_op_(_bool_op) {}

  virtual void select_overlap_faces(const OverlapFces& _overlap_faces);
  virtual void select_faces(Topo::Wrap<Topo::Type::BODY>& _body_a,
    Topo::Wrap<Topo::Type::BODY>& _body_b);

private:
  void propagate(const Choice _choice, Topo::Wrap<Topo::Type::FACE> _face);
  void apply_selection();

  std::set<Topo::Wrap<Topo::Type::FACE>> proc_faces_;
  std::set<Topo::Wrap<Topo::Type::FACE>> faces_to_remove_;
  std::set<Topo::Wrap<Topo::Type::FACE>> faces_to_invert_;
  std::set<Topo::Wrap<Topo::Type::EDGE>> common_edges_;
  Operation bool_op_;
};

MAKE_ENUM(FaceClassification, IN, OUT, OVERLAP, ANTIOVERLAP);

const Choice selection_table[2][Operation::ENUM_SIZE][FaceClassification::ENUM_SIZE] =
{
  // Selection first solid
  {// In    Out   Ovrl  AntiOvrlp
    { REMV, KEEP, KEEP, REMV }, // Union
    { KEEP, REMV, KEEP, REMV }, // Intersection
    { REMV, KEEP, REMV, KEEP }  // Difference
  },
  // Selection second solid
  {// In    Out   Ovrl  AntiOvrlp
    { REMV, KEEP, REMV, REMV }, // Union
    { KEEP, REMV, REMV, REMV }, // Intersection
    { INVR, REMV, REMV, REMV }  // Difference
  }
};

void Selection::select_overlap_faces(const OverlapFces& _overlap_faces)
{
  THROW_IF(_overlap_faces[0].size() != _overlap_faces[1].size(), "Overlap faces not coupled");
  std::vector<bool> used_faces(_overlap_faces[1].size(), false);
  for (size_t i = 0; i < _overlap_faces[0].size(); ++i)
  {
    bool processed = false;
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it_0(_overlap_faces[0][i]);
    for (size_t j = 0; j < _overlap_faces[1].size(); ++j)
    {
      if (used_faces[j])
        continue;
      Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it_1(_overlap_faces[1][j]);
      if (fv_it_0.size() != fv_it_1.size())
        continue;
      auto it_0 = fv_it_0.begin();
      auto it_1 = fv_it_1.begin();
      for (; it_1 != fv_it_1.end(); ++it_1)
      {
        if (*it_0 == *it_1)
          break;
      }
      if (it_1 == fv_it_1.end())
        continue; // no common vertices;

      bool overlap = true;
      bool anti_overlap = true;
      auto it_1_r = it_1;
      while (++it_0 != fv_it_0.end())
      {
        if (overlap)
        {
          if (++it_1 == fv_it_1.end())
            it_1 = fv_it_1.begin();
          overlap &= *it_1 == *it_0;
        }
        if (anti_overlap)
        {
          if (it_1_r == fv_it_1.begin())
            it_1_r = std::prev(fv_it_1.end());
          else
            --it_1_r;
          anti_overlap &= *it_1_r == *it_0;
        }
      }

      FaceClassification fc;
      if (overlap)
        fc = FaceClassification::OVERLAP;
      else if (anti_overlap)
        fc = FaceClassification::ANTIOVERLAP;
      else
        continue;
      for (size_t k = 0; k < 2; ++k)
      {
        auto& curr_face = _overlap_faces[k][k == 0 ? i : j];
        auto choice = selection_table[k][size_t(bool_op_)][size_t(fc)];
        if (choice == REMV)
          faces_to_remove_.insert(curr_face);
        else if (choice == INVR)
          faces_to_invert_.insert(curr_face);
        proc_faces_.insert(curr_face);
      }

      // Assuming that if face_i and face_j overlaps, it is not possible to have
      // other overlaps involving face_i and face_j. 
      // I am not sure this is always true.
      used_faces[j] = true;
      processed = true;
      break;
    }
    THROW_IF(!processed, "Impossible to find the match of an overlap face");
  }
}

void Selection::select_faces(
  Topo::Wrap<Topo::Type::BODY>& _body_a,
  Topo::Wrap<Topo::Type::BODY>& _body_b)
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> edges_a(_body_a);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> edges_b(_body_b);
  std::set<Topo::Wrap<Topo::Type::EDGE>> edge_sets[2];
  edge_sets[0].insert(edges_a.begin(), edges_a.end());
  edge_sets[1].insert(edges_b.begin(), edges_b.end());
  std::set_intersection(
    edge_sets[0].begin(), edge_sets[0].end(),
    edge_sets[1].begin(), edge_sets[1].end(),
    std::inserter(common_edges_, common_edges_.end()));
  for (auto& edge : common_edges_)
  {
    struct CoedgeVectors
    {
      Geo::Point coe_dir_, face_norm_, face_inside_dir_;
      bool processed_ = false;
      Topo::Wrap<Topo::Type::FACE> face_;
    };

    std::vector<CoedgeVectors> coe_vects[2];
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::COEDGE> ec_it(edge);
    for (auto& coed : ec_it)
    {
      Topo::Iterator<Topo::Type::COEDGE, Topo::Type::FACE> cf_it(coed);
      THROW_IF(cf_it.size() < 1, "Coedge without faces.");
      THROW_IF(cf_it.size() > 1, "Coedge with too many faces.");
      auto face = cf_it.get(0);

      THROW_IF(face->size(Topo::Direction::Up) != 1, "Face not in a body?");

      size_t body_idx = 0;
      if (_body_a.get() == face->get(Topo::Direction::Up, 0))
        body_idx = 0;
      else if (_body_b.get() == face->get(Topo::Direction::Up, 0))
        body_idx = 1;
      else
        THROW("Face body not present");

      coe_vects[body_idx].emplace_back();
      auto& vcts = coe_vects[body_idx].back();

      Geo::Segment seg;
      coed->geom(seg);
      vcts.coe_dir_ = Topo::coedge_direction(coed);
      vcts.face_norm_ = Topo::face_normal(face);
      vcts.processed_ = proc_faces_.find(face) != proc_faces_.end();
      vcts.face_inside_dir_ = vcts.face_norm_ % vcts.coe_dir_;
      vcts.face_ = face;
    }
    if (coe_vects[0].size() != 2 || coe_vects[1].size() != 2)
    {
      static std::string err_mess;
      err_mess ="Not supporing open bodies - common edges must have 2 facesper body.";
      err_mess += std::to_string(coe_vects[0].size()) + " " + 
        std::to_string(coe_vects[1].size());
      THROW(err_mess.c_str());
    }
    for (int i = 0; i < 2; ++i)
    {
      for (int j = 0; j < 2; ++j)
      {
        // mark vcts[i][j].face_ using vcts[1-i][0] and vcts[1-i][1]
        if (proc_faces_.find(coe_vects[i][j].face_) != proc_faces_.end())
          continue;
        FaceClassification fc;
        auto sin_outside_angle = coe_vects[1 - i][0].face_norm_ * coe_vects[1 - i][1].face_inside_dir_;
        if (std::fabs(sin_outside_angle) < 1e-10)
        {
          // Parallel or anti-parallel case - Ignoring anti-parallel for now.
          THROW_IF(coe_vects[1 - i][0].face_norm_ * coe_vects[1 - i][0].face_norm_ < 0, "Antiparallel faces");
          if (coe_vects[i][j].face_inside_dir_ * coe_vects[1 - i][j].face_norm_ < 0)
            fc = FaceClassification::IN;
          else 
            fc = FaceClassification::OUT;
        }
        else
        {
          double u, v;
          if (!Geo::decompose(
            coe_vects[i][j].face_inside_dir_,
            coe_vects[1 - i][0].face_inside_dir_,
            coe_vects[1 - i][1].face_inside_dir_, u, v))
          {
            continue;
          }
          bool out_face = (u > 0 && v > 0) ^ (sin_outside_angle < 0);
          fc = out_face ? FaceClassification::OUT : FaceClassification::IN;
        }
        Choice choice;
        choice = selection_table[i][size_t(bool_op_)][size_t(fc)];
        propagate(choice, coe_vects[i][j].face_);
      }
    }
  }
  apply_selection();
}

void Selection::propagate(const Choice _choice, Topo::Wrap<Topo::Type::FACE> _face)
{
  std::list<Topo::Wrap<Topo::Type::FACE>> face_list;
  face_list.push_back(_face);
  while (!face_list.empty())
  {
    auto face = face_list.front();
    if (_choice == INVR)
      faces_to_invert_.insert(face);
    else if (_choice == REMV)
      faces_to_remove_.insert(face);
    proc_faces_.insert(face);
    face_list.pop_front();
    Topo::Iterator<Topo::Type::FACE, Topo::Type::EDGE> fe_it(face);
    for (auto edge : fe_it)
    {
      if (common_edges_.find(edge) != common_edges_.end())
        continue;
      Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> ef_it(edge);
      for (auto face_new : ef_it)
      {
        if (proc_faces_.find(face_new) != proc_faces_.end())
          continue;
        face_list.push_front(face_new);
      }
    }
  }
}

void Selection::apply_selection()
{
  for (auto face : faces_to_remove_)
  {
    face->remove();
  }
  for (auto face : faces_to_invert_)
  {
    face->reverse();
  }
}

}//namespace

std::shared_ptr<ISelection> ISelection::make(Operation _bool_op)
{
  return std::make_shared<Selection>(_bool_op);
}

}//namespace Boolean
