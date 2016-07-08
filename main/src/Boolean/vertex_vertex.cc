
#include "priv.hh"

#include "Geo/vector.hh"
#include "Geo/minsphere.hh"
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
  MergeSets mrg_sets;
  for (size_t i = 0; i < _vert_it_a.size(); ++i)
  {
    for (size_t j = 0; j < _vert_it_b.size(); ++j)
    {
      Topo::Wrap<Topo::Type::VERTEX> va = _vert_it_a.get(i);
      Topo::Wrap<Topo::Type::VERTEX> vb = _vert_it_b.get(j);
      Geo::Point pt_a, pt_b;
      va->geom(pt_a);
      vb->geom(pt_b);
      auto tol = std::max(va->tolerance(), vb->tolerance());
      if (!Geo::same(pt_a, pt_b, tol))
        continue;

      auto it_a = find(mrg_sets, va);
      auto it_b = find(mrg_sets, vb);
      bool found_a = it_a != mrg_sets.end();
      bool found_b = it_b != mrg_sets.end();
      if (found_a != found_b)
      {
        if (found_b)
        {
          it_a = it_b;
          vb = va;
        }
        MergeSet tmp(*it_a);
        tmp.insert(vb);
        mrg_sets.erase(it_a);
        mrg_sets.insert(tmp);
      }
      else if (found_a)
      {
        if (it_a != it_b)
        {
          MergeSet mrg_set(*it_a);
          mrg_set.insert(it_b->begin(), it_b->end());
          mrg_sets.erase(it_b);
          mrg_sets.erase(it_a);
          mrg_sets.insert(mrg_set);
        }
      }
      else
      {
        MergeSet mrg_set;
        mrg_set.insert(va);
        mrg_set.insert(vb);
        mrg_sets.insert(mrg_set);
      }
    }
  }
  if (mrg_sets.empty())
    return false;
  for (const auto& mrg_set : mrg_sets)
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
