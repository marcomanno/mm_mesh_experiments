#pragma once

#include <array>
#include <set>

namespace Geo {

template<size_t DegT>
std::multiset<double> polygon_roots(const std::array<double, DegT + 1>& _poly);

} // namespace Geo
