#include <Topology/impl.hh>

#include <functional>

namespace UnitTest
{

typedef std::function<double(size_t, size_t)> IndexToPoint;

double cube_00(size_t i, size_t _i_xyz); // Unit cube in (0, 1), (0, 1), (0, 1)

double cube_01(size_t i, size_t _i_xyz); // Unit cube in (0, -1), (0, 1), (0, 1)

double cube_02(size_t i, size_t _i_xyz); // Unit cube in (-05, 05), (0, 1), (0, 1)

double cube_03(size_t i, size_t _i_xyz); // Unit cube in (-0.5, 0.5), (-0.5, 0.5), (0, 1)

double cube_04(size_t i, size_t _i_xyz); // Unit cube in (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)

Topo::Wrap<Topo::Type::BODY> make_cube(IndexToPoint idx_to_pt);

void print_body(Topo::Wrap<Topo::Type::BODY> _body);

typedef std::function<double(size_t, size_t)> IndexToPoint;
}
