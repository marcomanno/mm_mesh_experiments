#include "priv.hh"
#include "Geo/vector.hh"
//#ifdef DEB_ON
#include "Import/import.hh"
//#endif
#include "Topology/geom.hh"
#include "Topology/same.hh"
#include "Topology/shared.hh"

#include "Utils/error_handling.hh"

#include <fstream>
#include <list>
#include <set>

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
    { REMV, KEEP, REMV, KEEP }, // Difference
    { KEEP, KEEP, KEEP, KEEP }, // Split
    { KEEP, KEEP, KEEP, KEEP }, // SplitA
    { REMV, REMV, REMV, REMV }  // SplitB
  },
  // Selection second solid
  {// In    Out   Ovrl  AntiOvrlp
    { REMV, KEEP, REMV, REMV }, // Union
    { KEEP, REMV, REMV, REMV }, // Intersection
    { INVR, REMV, REMV, REMV }, // Difference
    { KEEP, KEEP, REMV, REMV }, // Split
    { REMV, REMV, REMV, REMV }, // SplitA
    { KEEP, KEEP, KEEP, KEEP }  // SplitB
  }
};

void Selection::select_overlap_faces(const OverlapFces& _overlap_faces)
{
  auto overlap_faces = _overlap_faces;
  for (auto& ovr_fa : overlap_faces)
  {
    for (auto ff_it = ovr_fa.end(); ff_it != ovr_fa.begin(); )
    {
      if ((*--ff_it)->size(Topo::Direction::Up) == 0)
        ff_it = ovr_fa.erase(ff_it);
    }
  }
  THROW_IF(overlap_faces[0].size() != overlap_faces[1].size(),
    "Overlap faces not coupled");
  std::vector<bool> used_faces(overlap_faces[1].size(), false);
#ifdef DEB_ON
  for (auto& f : overlap_faces[0])
  {
    static int f_ind = 1000;
    IO::save_face(f.get(), f_ind++);
  }
  for (auto& f : overlap_faces[1])
  {
    static int f_ind = 2000;
    IO::save_face(f.get(), f_ind++);
  }
#endif
  for (size_t i = 0; i < overlap_faces[0].size(); ++i)
  {
    bool processed = false;
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it_0(overlap_faces[0][i]);
    auto check_null_face = [this](
      const Topo::Iterator<Topo::Type::FACE,Topo::Type::VERTEX>& _fv_it,
      const Topo::Wrap<Topo::Type::FACE>& _f)
    {
      if (_fv_it.size() != 0)
        return false;
      faces_to_remove_.insert(_f);
      proc_faces_.insert(_f);
      return true;
    };
    if (check_null_face(fv_it_0, overlap_faces[0][i]))
    {
      processed = true;
      continue;
    }

    for (size_t j = 0; j < overlap_faces[1].size(); ++j)
    {
      if (used_faces[j])
        continue;
      Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it_1(overlap_faces[1][j]);
      if (check_null_face(fv_it_1, overlap_faces[1][j]))
      {
        used_faces[j] = true;;
        continue;
      }
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
            it_1_r = fv_it_1.end();
          anti_overlap &= (*--it_1_r) == *it_0;
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
        auto& curr_face = overlap_faces[k][k == 0 ? i : j];
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

//#ifdef DEB_ON
  static int nnnn = 0;
  std::ofstream deb_cmm_verts(std::string("deb_cmm_verts_") + std::to_string(nnnn++) + ".obj");
  for (auto& edge : common_edges_)
  {
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev_it(edge);
    Geo::Point pt;
    for (auto i : { 0, 1 })
    {
      ev_it.get(i)->geom(pt);
      deb_cmm_verts << "v " << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
    }
    deb_cmm_verts << "v " << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
  }
  for (int i = 1; i <= common_edges_.size(); ++i)
  {
    auto v1 = 3 * i;
    deb_cmm_verts << "f " << v1 << " " << v1 - 1 << " " << v1 - 2 << std::endl;
  }

#ifdef DEB_ON

  // finds a gap in the edges between two bodies. A gap may cause an
  // error and it is a bug with closed bodies.
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;
  for (auto& edge : common_edges_)
  {
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev_it(edge);
    verts.emplace_back(ev_it.get(0));
    verts.emplace_back(ev_it.get(1));
  }
  Topo::Wrap<Topo::Type::VERTEX> sten[2];
  while (!verts.empty())
  {
    sten[0] = verts.back(); verts.pop_back();
    sten[1] = verts.back(); verts.pop_back();
    for (int i = 0; i < 2; ++i)
    {
      for (;;)
      {
        auto pos = std::find(verts.begin(), verts.end(), sten[i]);
        if (pos == verts.end())
          break;
        auto n = pos - verts.begin();
        n ^= 1;
        sten[i] = verts[n];
        n &= ~1;
        verts.erase(verts.cbegin() + n, verts.cbegin() + n + 2);
      }
    }
    if (sten[0] == sten[1])
      continue;
    
    std::cout << "Open chain " << sten[0]->id() <<
      " " << sten[1]->id() << std::endl;
    for (int ii = 0; ii < 2; ++ii)
    {
      Topo::Iterator<Topo::Type::VERTEX, Topo::Type::FACE> vf(sten[ii]);
      static int nn = 0;
      for (auto& f : vf)
        IO::save_face(f.get(), nn++, false);
      auto bad_faces =
        Topo::shared_entities<Topo::Type::VERTEX, Topo::Type::FACE>(sten[0], sten[1]);
      for (auto f : bad_faces)
        std::cout << f->id() << " ";
      std::cout << "\n";
    }
  }
#endif

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
      std::cout << "Strange split\n";
    if (bool_op_ == Operation::SPLIT)
      continue;
    if (bool_op_ == Operation::SPLITA || bool_op_ == Operation::SPLITB)
    {
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < coe_vects[i].size(); ++j)
        {
          auto choice = selection_table[i][size_t(bool_op_)][0];
          propagate(choice, coe_vects[i][j].face_);
        }
      continue;
    }

#if 0
    // Remove completely overlapping faces on the same body.
    for (auto& coe_infos : coe_vects)
    {
      for (size_t i = 0; i < coe_infos.size(); )
      {
        bool reversed = false;
        for (size_t j = i + 1; j < coe_infos.size(); )
        {
          if (!Topo::same(coe_infos[i].face_, coe_infos[j].face_, &reversed))
            ++j;
          else
          {
            coe_infos[j].face_->remove();
            coe_infos.erase(coe_infos.begin() + j);
            if (reversed)
            {
              //std::cout << "Antioverlap: " << coe_infos[j].face_->id();
              //std::cout << " " << coe_infos[i].face_->id() << "\n";
              break;
            }
          }
        }
        if (!reversed)
          ++i;
        else
        {
          coe_infos[i].face_->remove();
          coe_infos.erase(coe_infos.begin() + i);
        }
      }
    }
#endif
    if (coe_vects[0].empty() && coe_vects[1].empty())
      continue;
    if (coe_vects[0].size() != 2 || coe_vects[1].size() != 2)
    {
      static std::string err_mess;
      err_mess ="Not supporing open bodies - common edges must have 2 faces per body.";
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
  std::vector<Topo::IBase*> tmp_face_to_remove;
  for (auto f : faces_to_remove_)
    tmp_face_to_remove.push_back(f.get());

  _body_a->remove_children(tmp_face_to_remove);
  _body_b->remove_children(tmp_face_to_remove);
  apply_selection();
}

void Selection::propagate(const Choice _choice, Topo::Wrap<Topo::Type::FACE> _face)
{
  std::list<Topo::Wrap<Topo::Type::FACE>> face_list;
  face_list.push_back(_face);
  proc_faces_.insert(_face);
  while (!face_list.empty())
  {
    auto face = face_list.front();
    if (_choice == INVR)
      faces_to_invert_.insert(face);
    else if (_choice == REMV)
      faces_to_remove_.insert(face);
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
        proc_faces_.insert(face_new);
        face_list.push_front(face_new);
      }
    }
  }
}

void Selection::apply_selection()
{
  for (auto face : faces_to_remove_)
  {
     THROW_IF(!face->remove(), "Aldready removedface!");
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
