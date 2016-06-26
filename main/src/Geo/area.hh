#pragma once
#include "vector.hh"
#include <vector>

namespace Geo
{
inline double area_compute(const Geo::Vector3& _v0, const Geo::Vector3& _v1)
{
  return Geo::length(_v0 % _v1) / 2;
}

inline double area_compute(const Geo::Vector3& _a,
  const Geo::Vector3& _b, const Geo::Vector3& _c)
{
  return area_compute(_b - _a, _c - _a);
}

inline double area_compute(const std::array<Geo::Vector3, 3 >& _tri)
{
  return area_compute(_tri[0], _tri[1], _tri[2]);
}

double area_compute(const std::vector<std::array<Geo::Vector3, 3>>& _tris);

}