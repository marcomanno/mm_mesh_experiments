#pragma once

#include "vector.hh"

#include <memory>
#include <vector>

namespace Geo
{

struct LinkingNumber
{
  static int compute(
    const std::vector<Geo::Vector3>& _loop0, 
    const std::vector<Geo::Vector3>& _loop1);
};


} // namespace Geo
