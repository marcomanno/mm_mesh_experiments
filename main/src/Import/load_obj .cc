#include <import.hh>

#include <Geo/vector.hh>

#include <Topology/impl.hh>


#include <fstream>
#include <sstream>

namespace Import {

Topo::Wrap<Topo::Type::BODY> load_obj(const char* _flnm)
{
  Topo::Wrap<Topo::Type::BODY> new_body;
  std::ifstream fstr(_flnm);
  std::string line;
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;

  new_body.make<Topo::EE<Topo::Type::BODY>>();
  while (std::getline(fstr, line))
  {
    if (line.size() < 3 || line[1] != ' ')
      continue;
    std::istringstream buf(line.c_str() + 2);
    if (line[0] == 'v')
    {
      Geo::Point pt;
      for (auto& coord : pt)
        buf >> coord;
      verts.emplace_back();
      verts.back().make<Topo::EE<Topo::Type::VERTEX>>();
      verts.back()->set_geom(pt);
    }
    else if (line[0] == 'f')
    {
      Topo::Wrap<Topo::Type::FACE> face;
      face.make<Topo::EE<Topo::Type::FACE>>();
      new_body->insert_child(face.get());
      size_t vert_idx;
      while (buf >> vert_idx)
      {
        face->insert_child(verts[vert_idx - 1].get());
        char c;
        while (buf >> c && c != ' ' && buf >> vert_idx);
      }
    }
  }
  return new_body;
}

}//namespace Import