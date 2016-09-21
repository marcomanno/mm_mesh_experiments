#include "topology_help.hh"

#include <Topology/iterator.hh>
#include <Geo/vector.hh>

#include <iostream>

namespace UnitTest
{

double cube_00(size_t i, size_t _i_xyz) // Unit cube in (0, 1), (0, 1), (0, 1)
{
  return (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
};

double cube_01(size_t i, size_t _i_xyz) // Unit cube in (0, -1), (0, 1), (0, 1)
{
  auto coord = (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
  if (_i_xyz == 0)
    coord *= -1;
  return coord;
};

double cube_02(size_t i, size_t _i_xyz) // Unit cube in (-05, 05), (0, 1), (0, 1)
{
  auto coord = (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
  if (_i_xyz == 0)
    coord -= 0.5;
  return coord;
};

double cube_03(size_t i, size_t _i_xyz) // Unit cube in (-0.5, 0.5), (-0.5, 0.5), (0, 1)
{
  auto coord = (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
  if (_i_xyz != 2)
    coord -= 0.5;
  return coord;
};

Topo::Wrap<Topo::Type::BODY> make_cube(IndexToPoint idx_to_pt)
{
  Topo::Wrap<Topo::Type::BODY> body;
  auto bb = body.make<Topo::EE<Topo::Type::BODY>>();

  Topo::Wrap<Topo::Type::VERTEX> verts[8];
  for (size_t i = 0; i < std::size(verts); ++i)
  {
    auto vert = verts[i].make<Topo::EE<Topo::Type::VERTEX>>();
    vert->set_geom({ idx_to_pt(i, 0), idx_to_pt(i, 1), idx_to_pt(i, 2) });
    vert->set_tolerance(1e-15);
  }

  size_t face_inds[][4] =
  {
    { 4, 5, 7, 6 },
    { 0, 2, 3, 1 },
    { 0, 1, 5, 4 },
    { 2, 6, 7, 3 },
    { 1, 3, 7, 5 },
    { 0, 4, 6, 2 }
  };

  Topo::Wrap<Topo::Type::FACE> faces[6];
  size_t f_idx = 0;
  for (auto& f : faces)
  {
    auto face = f.make<Topo::EE<Topo::Type::FACE>>();
    bb->insert_child(face);
    size_t * p = face_inds[f_idx++];
    for (size_t j = 0; j < 4; ++j)
    {
      auto& v = verts[p[j]];
      auto v_ptr = dynamic_cast<Topo::EE<Topo::Type::VERTEX>*>(v.get());
      face->insert_child(v_ptr);
    }
  }
  return body;
}

void print_body(Topo::Wrap<Topo::Type::BODY> _body)
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf;
  bf.reset(_body);
  for (size_t i = 0; i < bf.size(); ++i)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fe;
    fe.reset(bf.get(i));
    std::cout << "Face " << i << std::endl;
    for (size_t j = 0; j < fe.size(); ++j)
    {
      Geo::Point pt;
      fe.get(j)->geom(pt);
      std::cout << "Pt" << j << " = " << pt << std::endl;
    }
  }
}

}
