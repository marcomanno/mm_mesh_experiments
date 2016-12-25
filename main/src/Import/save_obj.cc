#include <import.hh>

#include <Geo/vector.hh>

#include <Topology/impl.hh>
#include <Topology/iterator.hh>
#include <PolygonTriangularization/poly_triang.hh>

#include <fstream>
#include <iomanip>
#include <sstream>

namespace IO {

#define SPLIT
bool save_obj(const char* _flnm, const Topo::Wrap<Topo::Type::BODY> _body)
{
  std::ofstream fstr(_flnm);
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
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> ve_it(f);
#ifndef SPLIT
    fstr << "f";
#else
    std::vector<Geo::Vector3> plgn;
#endif
    for (const auto& v : ve_it)
    {
#ifndef SPLIT
      Geom::Point pt;
      v->geom(pt);
      auto idx = std::lower_bound(all_pts.begin(),
        all_pts.end(), pt) - all_pts.begin() + 1;
      fstr << " " << idx;
#else
      plgn.emplace_back();
      v->geom(plgn.back());
#endif
    }
#ifndef SPLIT
    fstr << "\n";
#else
    auto poly_t = IPolygonTriangulation::make();
    poly_t->add(plgn);
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
#endif
  }
  return fstr.good();
}

bool save_face(const Topo::E<Topo::Type::FACE>* _ptr, int _num,
  const bool _split)
{
  std::ofstream fstr(std::to_string(_num) + ".obj");
  fstr << std::setprecision(17);
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> vert_it(
    const_cast<Topo::E<Topo::Type::FACE>*>(_ptr));
  if (!_split)
  {
    std::vector<Geo::Vector3> plgn;
    for (const auto& vert : vert_it)
    {
      Geo::Vector3 pt;
      vert->geom(pt);
      plgn.emplace_back(pt);
    }
    auto poly_t = IPolygonTriangulation::make();
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
    std::string fv("f");
    int i = 0;
    for (const auto& vert : vert_it)
    {
      Geo::Point pt;
      vert->geom(pt);
      fstr << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
      fv += ' ';
      fv += std::to_string(++i);
    }
    fstr << fv;
  }
  return fstr.good();
}

}//namespace Import