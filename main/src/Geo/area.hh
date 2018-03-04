#pragma once
#include "vector.hh"
#include <vector>

namespace Geo
{
inline double area(const Geo::VectorD3& _v0, const Geo::VectorD3& _v1)
{
  return Geo::length(_v0 % _v1) / 2;
}

inline double area(const Geo::VectorD3& _a,
  const Geo::VectorD3& _b, const Geo::VectorD3& _c)
{
  return area(_b - _a, _c - _a);
}

inline double area(const std::array<Geo::VectorD3, 3 >& _tri)
{
  return area(_tri[0], _tri[1], _tri[2]);
}

double area(const std::vector<std::array<Geo::VectorD3, 3>>& _tris);

}