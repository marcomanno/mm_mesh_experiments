#include <import.hh>

#include <Geo/vector.hh>

#include <PolygonTriangularization/poly_triang.hh>
#include <Topology/impl.hh>
#include <Topology/iterator.hh>
#include <Utils/error_handling.hh>

#include <fstream>
#include <iomanip>
#include <sstream>

namespace IO {

#define SPLIT
bool save_obj(const char* _flnm, const Topo::Wrap<Topo::Type::BODY> _body)
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
    auto poly_t = Geo::IPolygonTriangulation::make();
    for (const auto& loop : fl_it)
    {
      std::vector<Geo::Vector3> plgn;
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
    std::vector<Geo::Vector3> plgn;
    for (const auto& vert : vert_it)
    {
      Geo::Vector3 pt;
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
  const std::vector<Geo::Vector3>& _plgn,
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
  std::vector<Geo::Vector3> plgn;
  for (auto vert : _verts)
  {
    plgn.emplace_back();
    vert->geom(plgn.back());
  }
  save_obj(_flnm, plgn);
}

}//namespace Import