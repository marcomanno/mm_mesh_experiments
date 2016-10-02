#include <import.hh>

#include <Geo/vector.hh>

#include <Topology/impl.hh>
#include <Topology/iterator.hh>


#include <fstream>
#include <sstream>

namespace Import {

bool save_obj(const char* _flnm, const Topo::Wrap<Topo::Type::BODY> _body)
{
  std::ofstream fstr(_flnm);
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
    fstr << "f";
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> ve_it(f);
    for (size_t j = 0; j < ve_it.size(); ++j)
    {
      auto v = ve_it.get(j);
      auto idx = 
        std::lower_bound(verts.begin(), verts.end(), v) - verts.begin() + 1;
      fstr << " " << idx;
    }
    fstr << "\n";
  }
  return fstr.good();
}

}//namespace Import