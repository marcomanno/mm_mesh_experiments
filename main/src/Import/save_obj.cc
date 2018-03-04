//#pragma optimize ("", off)
#include <import.hh>

#include <Geo/vector.hh>

#include <PolygonTriangularization/poly_triang.hh>
#include <Topology/impl.hh>
#include <Topology/iterator.hh>
#include <Utils/error_handling.hh>

#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>

namespace IO {

bool save_obj(const char* _flnm, const Topo::Wrap<Topo::Type::BODY> _body,
              bool _split)
{
  std::ofstream fstr(_flnm);
  THROW_IF(!fstr, "IO save error");
  fstr << std::setprecision(17);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> vert_it(_body);
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;
  for (size_t i = 0; i < vert_it.size(); ++i)
  {
    auto v = vert_it.get(i);
    verts.emplace_back(v);
  }
  std::sort(verts.begin(), verts.end());
  verts.erase(std::unique(verts.begin(), verts.end()), verts.end());
  std::vector<Geo::Point> all_pts;
  for (const auto& v : verts)
  {
    all_pts.emplace_back();
    v->geom(all_pts.back());
  }
  std::sort(all_pts.begin(), all_pts.end());

  for (const auto& pt : all_pts)
    fstr << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> face_it(_body);
  for (size_t i = 0; i < face_it.size(); ++i)
  {
    auto f = face_it.get(i);
    Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> fl_it(f);
    if (_split)
    {
      auto poly_t = Geo::IPolygonTriangulation::make();
      for (const auto& loop : fl_it)
      {
        std::vector<Geo::VectorD3> plgn;
        Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> lv_it(loop);
        for (const auto& v : lv_it)
        {
          plgn.emplace_back();
          v->geom(plgn.back());
        }
        poly_t->add(plgn);
      }
      for (const auto& tri : poly_t->triangles())
      {
        fstr << "f";
        for (auto ind : tri)
        {
          const auto& pt = poly_t->polygon()[ind];
          const auto idx = std::lower_bound(all_pts.begin(),
                                            all_pts.end(), pt) - all_pts.begin() + 1;
          fstr << " " << idx;
        }
        fstr << "\n";
      }
    }
    else
    {
      bool isle = false;
      for (const auto& loop : fl_it)
      {
        Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> lv_it(loop);
        fstr << "f";
        if (isle)
          fstr << "  ";

        for (const auto& v : lv_it)
        {
          Geo::Point pt;
          v->geom(pt);
          const auto idx = std::lower_bound(
            all_pts.begin(), all_pts.end(), pt) - all_pts.begin() + 1;
          fstr << " " << idx;
        }
        fstr << "\n";
        isle = true;
      }
    }
  }
  return fstr.good();
}

bool save_face(const Topo::E<Topo::Type::FACE>* _ptr, const char* _flnm,
  const bool _split)
{
  std::ofstream fstr(_flnm);
  fstr << std::setprecision(17);
  if (_split)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> vert_it(
      const_cast<Topo::E<Topo::Type::FACE>*>(_ptr));
    std::vector<Geo::VectorD3> plgn;
    for (const auto& vert : vert_it)
    {
      Geo::VectorD3 pt;
      vert->geom(pt);
      plgn.emplace_back(pt);
    }
    auto poly_t = Geo::IPolygonTriangulation::make();
    poly_t->add(plgn);
    for (const auto& pt : poly_t->polygon())
    {
      fstr << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
    }
    for (const auto& tri : poly_t->triangles())
    {
      fstr << "f";
      for (auto idx : tri)
        fstr << " " << idx + 1;
      fstr << "\n";
    }
  }
  else
  {
    std::string fv;
    int i = 0;
    Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> loop_it(
      const_cast<Topo::E<Topo::Type::FACE>*>(_ptr));
    for (const auto& loop : loop_it)
    {
      Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> vert_it(loop);
      fv += "f";
      for (const auto& vert : vert_it)
      {
        Geo::Point pt;
        vert->geom(pt);
        fstr << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
        fv += ' ';
        fv += std::to_string(++i);
      }
      fv += "\n";
    }
    fstr << fv;
  }
  return fstr.good();
}

bool save_face(const Topo::E<Topo::Type::FACE>* _ptr, int _num,
               const bool _split)
{
  return save_face(_ptr, (std::to_string(_num) + ".obj").c_str(), _split);
}

void save_obj(const char* _flnm, 
  const std::vector<Geo::VectorD3>& _plgn,
  const std::vector<size_t>* _inds)
{
  std::string flnm;
  static int n = 0;
  if (_flnm != nullptr)
    flnm = _flnm;
  else
    flnm = std::string("_deb_obj_") + std::to_string(n++);
  std::ofstream ff(flnm + ".obj");
  ff << std::setprecision(17);
  for (const auto& v : _plgn) { ff << "v" << v << "\n"; }
  
  ff << "f";
  if (_inds == nullptr)
    for (int i = 1; i <= _plgn.size(); ++i)
      ff << " " << i;
  else
    for (auto i : *_inds)
      ff << " " << i + 1;
  ff << "\n";
}

void save_obj(const char* _flnm,
  const std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _verts)
{
  std::vector<Geo::VectorD3> plgn;
  for (auto vert : _verts)
  {
    plgn.emplace_back();
    vert->geom(plgn.back());
  }
  save_obj(_flnm, plgn);
}

struct Saver : public ISaver
{
  void add_face(const Topo::Wrap<Topo::Type::FACE>& _f) override { faces_.insert(_f); }
  void add_edge(const Topo::Wrap<Topo::Type::EDGE>& _e) override { edges_.insert(_e); }
  bool compute(const char* _flnm) override;
  std::set<Topo::Wrap<Topo::Type::FACE>> faces_;
  std::set<Topo::Wrap<Topo::Type::EDGE>> edges_;
};

std::shared_ptr<ISaver> ISaver::make()
{
  return std::make_shared<Saver>();
}

bool Saver::compute(const char* _flnm)
{
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> vertices;
  for (const auto& f : faces_)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> vert_it(f);
    for (auto& vert : vert_it)
      vertices.push_back(vert);
  }
  for (const auto& e : edges_)
  {
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ed_it(e);
    for (auto& vert : ed_it)
      vertices.push_back(vert);
  }
  std::sort(vertices.begin(), vertices.end());
  auto new_end = std::unique(vertices.begin(), vertices.end());
  vertices.erase(new_end, vertices.end());
  std::ofstream fstr(_flnm);
  fstr << std::setprecision(17);
  auto save_vertices = [&fstr](std::vector<Topo::Wrap<Topo::Type::VERTEX>>& _vv)
  {
    for (auto& v : _vv)
    {
      Geo::Point pt;
      v->geom(pt);
      fstr << "v " << pt << "\n";
    }
  };
  auto vertex_position = [&vertices](const Topo::Wrap<Topo::Type::VERTEX>& _v)
  {
    auto range = std::equal_range(vertices.begin(), vertices.end(), _v);
    if (range.first == range.second)
      return ptrdiff_t(-1);
    return std::distance(vertices.begin(), range.first);
  };
  std::vector<std::array<size_t, 3>> inters;
  std::vector<size_t> extra_ind(vertices.size(), 0);
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> extra_vertices;
  for (const auto& e : edges_)
  {
    Topo::Iterator<Topo::Type::EDGE, Topo::Type::FACE> ef(e);
    bool skip = false;
    for (auto& f : ef)
      skip |= faces_.find(f) != faces_.end();
    if (!skip)
    {
      Topo::Iterator<Topo::Type::EDGE, Topo::Type::VERTEX> ev(e);
      auto it = ev.begin();
      auto v_ind0 = vertex_position(*it);
      auto v_ind1 = vertex_position(*++it);
      auto& v_ind2 = extra_ind[v_ind0];
      if (v_ind2 == 0)
      {
        v_ind2 = extra_vertices.size();
        extra_vertices.push_back(vertices[v_ind0]);
      }
      inters.push_back({ size_t(v_ind0) , size_t(v_ind1), v_ind2 });
    }
  }
  save_vertices(vertices);
  save_vertices(extra_vertices);

  for (const auto& f : faces_)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> fel(f);
    auto polyt = Geo::IPolygonTriangulation::make();
    std::map<Geo::Point, Topo::Wrap<Topo::Type::VERTEX>> pos_vert_map;
    for (const auto& loop : fel)
    {
      Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> lv(loop);
      std::vector<Geo::Point> plygon;
      for (const auto& v : lv)
      {
        Geo::Point pt;
        v->geom(pt);
        plygon.push_back(pt);
        pos_vert_map.emplace(pt, v);
      }
      polyt->add(plygon);
    }
    const auto& tris = polyt->triangles();
    const auto& pos = polyt->polygon();
    for (auto& tri : tris)
    {
      fstr << "f";
      for (int i = 0; i < 3; ++i)
      {
        const auto& v = pos_vert_map[pos[tri[i]]];
        auto v_ind = vertex_position(v);
        fstr << " " << v_ind + 1;
      }
      fstr << "\n";
    }
  }

  for (auto& tri : inters)
    fstr << "f " << tri[0] + 1 << " " << tri[1] + 1 << " " << tri[2] + vertices.size() + 1 << "\n";

  return true;
}


}//namespace Import