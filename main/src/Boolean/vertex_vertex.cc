#include "priv.hh"

#include "Geo/kdtree.hh"
#include "Geo/minsphere.hh"
#include "Geo/vector.hh"
#include "Utils/equivalence_relation.hh"
#include "Utils/statistics.hh"

#include <set>
#include <vector>

namespace Boolean {

namespace {

typedef std::set<Topo::Wrap<Topo::Type::VERTEX>> MergeSet;
typedef std::set<MergeSet> MergeSets;

MergeSets::iterator find(MergeSets& mrg_sets, Topo::Wrap<Topo::Type::VERTEX> _vert)
{
  MergeSet mrg_set;
  mrg_set.insert(_vert);
  auto it = mrg_sets.lower_bound(mrg_set);
  if (it != mrg_sets.end())
    ++it;
  while (it != mrg_sets.begin())
  {
    --it;
    if (it->find(_vert) != it->end())
      return it;
    if (*(it->rbegin()) < _vert)
      break;
  }
  return mrg_sets.end();
}

}

bool vertices_versus_vertices(
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it_a,
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX>& _vert_it_b)
{
  Geo::KdTree<Topo::Wrap<Topo::Type::VERTEX>> kdtree[2];
  kdtree[0].insert(_vert_it_a.begin(), _vert_it_a.end());
  kdtree[1].insert(_vert_it_b.begin(), _vert_it_b.end());
  kdtree[0].compute();
  kdtree[1].compute();

  std::vector<std::array<size_t, 2>> vert_couples =
    Geo::find_kdtree_couples<Topo::Wrap<Topo::Type::VERTEX>>(kdtree[0], kdtree[1]);

  Utils::EquivalenceRelations<Topo::Wrap<Topo::Type::VERTEX>> equiv_set;
  for (const auto& couple : vert_couples)
  {
    Topo::Wrap<Topo::Type::VERTEX> va = kdtree[0][couple[0]];
    Topo::Wrap<Topo::Type::VERTEX> vb = kdtree[1][couple[1]];
    Geo::Point pt_a, pt_b;
    va->geom(pt_a);
    vb->geom(pt_b);
    auto tol = std::max(va->tolerance(), vb->tolerance());
    if (!Geo::same(pt_a, pt_b, tol))
      continue;
    equiv_set.add_relation(va, vb);
  }
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> mrg_set;
  while (!(mrg_set = equiv_set.extract_equivalence_set()).empty())
  {
    std::vector<Geo::Point> pt_to_mrg;
    for (const auto& vert : mrg_set)
    {
      Geo::Point pt;
      vert->geom(pt);
      pt_to_mrg.push_back(pt);
    }
    auto sph = Geo::min_ball(pt_to_mrg.data(), pt_to_mrg.size());
    Utils::FindMax<double> max_tol;
    for (const auto& vert : mrg_set)
    {
      Geo::Point pt;
      vert->geom(pt);
      auto new_tol = Geo::length(pt - sph.centre_) + vert->tolerance();
      max_tol.add(new_tol);
    }
    auto vert0 = *mrg_set.begin();
    vert0->set_geom(sph.centre_);
    vert0->set_tolerance(max_tol());
    auto vert_it = mrg_set.begin();
    while (++vert_it != mrg_set.end())
    {
      auto vert = *vert_it;
      vert->replace(vert0.get());
    }
  }
  return true;
}

}//namespace Boolean
