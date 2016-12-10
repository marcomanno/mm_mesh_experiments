#include <import.hh>

#include <Geo/vector.hh>

#include <Topology/impl.hh>
#include <Topology/iterator.hh>
#include <PolygonTriangularization/poly_triang.hh>

#include <fstream>
#include <iomanip>
#include <sstream>

namespace IO {

//#define SPLIT
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
  for (const auto& v : verts)
  {
    Geo::Point pt;
    v->geom(pt);
    fstr << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
  }
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> face_it(_body);
  for (size_t i = 0; i < face_it.size(); ++i)
  {
    auto f = face_it.get(i);
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> ve_it(f);
    std::vector<Geo::Vector3> plgn;
    std::vector<size_t> idxs;
#ifndef SPLIT
    fstr << "f";
#endif
    for (const auto& v : ve_it)
    {
      plgn.emplace_back();
      v->geom(plgn.back());
      auto idx = std::lower_bound(
        verts.begin(), verts.end(), v) - verts.begin() + 1;
      idxs.push_back(idx);
#ifndef SPLIT
      fstr << " " << idx;
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
      for (auto idx : tri)
        fstr << " " << idxs[idx];
      fstr << "\n";
    }
#endif
  }
  return fstr.good();
}

bool save_face(Topo::E<Topo::Type::FACE>* _ptr, int _num)
{
  std::ofstream fstr(std::to_string(_num) + ".obj");
  fstr << std::setprecision(17);
  Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> vert_it(_ptr);
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
  return fstr.good();
}

}//namespace Import