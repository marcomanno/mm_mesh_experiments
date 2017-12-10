//#pragma optimize("", off)
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
  void apply_selection(std::array<Topo::Wrap<Topo::Type::BODY>, 2>& _bodies);

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
    { REMV, REMV, REMV, REMV }, // SplitB
    { KEEP, REMV, REMV, REMV }, // A_IN_B
    { REMV, REMV, REMV, REMV }, // B_IN_A
    { REMV, KEEP, REMV, REMV }, // A_OUT_B
    { REMV, REMV, REMV, REMV }, // B_OUT_A
    { REMV, REMV, KEEP, KEEP }, // A_OVERLAP
    { REMV, REMV, REMV, REMV }  // B_OVERLAP
  },
  // Selection second solid
  {// In    Out   Ovrl  AntiOvrlp
    { REMV, KEEP, REMV, REMV }, // Union
    { KEEP, REMV, REMV, REMV }, // Intersection
    { INVR, REMV, REMV, REMV }, // Difference
    { KEEP, KEEP, REMV, REMV }, // Split
    { REMV, REMV, REMV, REMV }, // SplitA
    { KEEP, KEEP, KEEP, KEEP },  // SplitB
    { REMV, REMV, REMV, REMV }, // A_IN_B
    { REMV, KEEP, REMV, REMV }, // B_IN_A
    { REMV, REMV, REMV, REMV }, // A_OUT_B
    { REMV, KEEP, REMV, REMV }, // B_OUT_A
    { REMV, REMV, REMV, REMV }, // A_OVERLAP
    { REMV, REMV, KEEP, KEEP }  // B_OVERLAP
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
  std::array<Topo::Wrap<Topo::Type::BODY>, 2> bodies = { _body_a , _body_b };
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> edges_a(_body_a);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> edges_b(_body_b);
  std::set<Topo::Wrap<Topo::Type::EDGE>> edge_sets[2];
  edge_sets[0].insert(edges_a.begin(), edges_a.end());
  edge_sets[1].insert(edges_b.begin(), edges_b.end());
  std::set_intersection(
    edge_sets[0].begin(), edge_sets[0].end(),
    edge_sets[1].begin(), edge_sets[1].end(),
    std::inserter(common_edges_, common_edges_.end()));


  static int nnnn = 0;
  auto saver = IO::ISaver::make();

  for (auto& face : faces_to_remove_)
    saver->add_face(face);
  for (auto& edge : common_edges_)
    saver->add_edge(edge);
  saver->compute((std::string("deb_cmm_verts_") + std::to_string(nnnn++) + ".obj").c_str());

#if 0
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
#endif

//#define DEB_ON
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
  typedef std::array<Topo::Wrap<Topo::Type::VERTEX>, 2> VertexCouple;
  VertexCouple sten;
  std::vector<VertexCouple> all_ends;
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

    auto print_face_info = [](const Topo::Wrap<Topo::Type::VERTEX>& _a,
                              const Topo::Wrap<Topo::Type::VERTEX>& _b)
    {
      auto bad_faces =
        Topo::shared_entities<Topo::Type::VERTEX, Topo::Type::FACE>(_a, _b);
      if (bad_faces.empty())
        return;
      std::cout << "Bad faces v" << _a->id() << " v" << _b->id();
      for (auto f : bad_faces)
      {
        std::cout << " f" << f->id();
        auto body = f->get(Topo::Direction::Up, 0);
        std::cout << " B" << body->id();
        if (body->id() == 32)
        {
          Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv(f);
          std::cout << std::endl;
          for (auto v : fv)
            std::cout << " " << v->id();
          std::cout << std::endl;
        }
      }
      std::cout << "\n";
    };
    for (auto& oth : all_ends)
      for (size_t ii = 0; ii < 4; ++ii)
        print_face_info(oth[ii % 2], sten[ii / 2]);
    all_ends.push_back(sten);

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
      auto face_body = face->get(Topo::Direction::Up, 0);
      if (bodies[body_idx].get() != face_body && bodies[++body_idx].get() != face_body)
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
#if 0
    if (coe_vects[0].size() != 2 || coe_vects[1].size() != 2)
      std::cout << "Strange split\n";
#endif
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
    auto process_face = [this, &coe_vects](
      FaceClassification _fc, Topo::Wrap<Topo::Type::FACE>& _face, size_t _i_solid)
    {
      if (proc_faces_.find(_face) != proc_faces_.end())
        return;
      Choice choice = selection_table[_i_solid][size_t(bool_op_)][size_t(_fc)];
      propagate(choice, _face);
    };

    if (coe_vects[0].size() == 1 && coe_vects[1].size() == 1)
    {
      FaceClassification fc = coe_vects[0][0].coe_dir_ * coe_vects[0][1].coe_dir_ < 0 ?
        FaceClassification::IN : FaceClassification::OUT;
      process_face(fc, coe_vects[0][0].face_, 0);
      process_face(fc, coe_vects[1][0].face_, 1);
      continue;
    }
    if (coe_vects[0].size() > 2 || coe_vects[1].size() > 2)
    {
      static std::string err_mess_root("Not supporing open bodies - common edges must have 2 faces per body.");
      auto err_mess = err_mess_root + std::to_string(coe_vects[0].size()) + " " +
        std::to_string(coe_vects[1].size());
      static int num = 0;
      for (auto& coe : coe_vects)
      {
        for (auto& f : coe)
        {
          IO::save_face(f.face_.get(), 
            (std::string("deb_selection_") + std::to_string(num++) + ".obj").c_str(), false);
        }
        num = 10 * (num / 10 + 1);
      }
      num = 100 * (num / 100 + 1);
      continue;
      //THROW(err_mess.c_str());
    }
    for (int i = 0; i < 2; ++i)
    {
      if (coe_vects[i].size() == 1)
      {
        double angs[2];
        Geo::iterate_forw<2>::eval([&angs, &coe_vects, i](size_t _j)
        {
          angs[_j] = Geo::signed_angle(coe_vects[i][0].face_inside_dir_, 
                                       coe_vects[1 - i][_j].face_inside_dir_,
                                       coe_vects[i][0].coe_dir_);
          if (angs[_j] < 0) angs[_j] += 2 * M_PI;
        });
        FaceClassification fc[2] = { FaceClassification::OUT , FaceClassification::IN };
        if (angs[0] > angs[1])
          std::swap(fc[0], fc[1]);
        for (int j = 0; j < 2; ++j)
          process_face(fc[j], coe_vects[1 - i][j].face_, 1 - i);
      }
      for (int j = 0; j < coe_vects[i].size(); ++j)
      {
        if (coe_vects[1 - i].size() != 2)
          continue;
        // mark vcts[i][j].face_ using vcts[1-i][0] and vcts[1-i][1]
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
        process_face(fc, coe_vects[i][j].face_, i);
      }
    }
  }

  auto remove_not_processed_faces = [this](Topo::Wrap<Topo::Type::BODY>& _body)
  {
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> f_it(_body);
    for (auto f : f_it)
      if (proc_faces_.find(f) == proc_faces_.end())
        faces_to_remove_.insert(f);
  };
  for (auto i : { 0,1 })
  {
    auto rem_in = selection_table[i][size_t(bool_op_)][size_t(FaceClassification::IN)];
    auto rem_out = selection_table[i][size_t(bool_op_)][size_t(FaceClassification::OUT)];
    if (rem_in == REMV && rem_out == REMV)
      remove_not_processed_faces(bodies[i]);
  }

  apply_selection(bodies);
}

void Selection::propagate(const Choice _choice, Topo::Wrap<Topo::Type::FACE> _face)
{
  std::list<Topo::Wrap<Topo::Type::FACE>> face_list;
  face_list.push_back(_face);
  proc_faces_.insert(_face);

  auto body = _face->get(Topo::Direction::Up, 0);

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
      for (auto new_face : ef_it)
      {
        if (proc_faces_.find(new_face) != proc_faces_.end())
          continue;
        auto body_f_new = new_face->get(Topo::Direction::Up, 0);
        if (body_f_new != body)
        {
          static int n;
          IO::save_face(new_face.get(), 
            (std::to_string(n++) + "_not_split_face").c_str(), true);
          continue;
        }
        proc_faces_.insert(new_face);
        face_list.push_front(new_face);
      }
    }
  }
}

void Selection::apply_selection(std::array<Topo::Wrap<Topo::Type::BODY>, 2>& _bodies)
{
  std::vector<Topo::IBase*> tmp_face_to_remove;
  for (auto f : faces_to_remove_)
    tmp_face_to_remove.push_back(f.get());

  _bodies[0]->remove_children(tmp_face_to_remove);
  _bodies[1]->remove_children(tmp_face_to_remove);

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
