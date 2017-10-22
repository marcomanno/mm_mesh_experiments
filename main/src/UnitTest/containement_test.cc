
#include "catch/catch.hpp"
#include "Utils/continement_tree.hh"

#include <array>
#include <queue>

namespace
{
using Interval = std::array<double, 2>;
int compare_intervals(const Interval& _a, const Interval& _b)
{
  if (_a == _b)
    return -1;

  if (_a[0] >= _b[0] && _a[1] <= _b[1])
    return -1;
  if (_b[0] >= _a[0] && _b[1] <= _a[1])
    return 1;
  return 0;
}

} // namespace

TEST_CASE("containement_basic_001", "[Containement]")
{
  Utils::ContainementTree<Interval> cnt_interv(compare_intervals);
  cnt_interv.add(Interval({0., 10.}));
  cnt_interv.add(Interval({ 1., 5. }));
  cnt_interv.add(Interval({ 6., 7. }));
  cnt_interv.add(Interval({ 8., 9. }));
  cnt_interv.add(Interval({ 2., 4. }));
  cnt_interv.add(Interval({ 2.5, 3. }));
  const auto& root = cnt_interv.root();
  auto el = root->child();
  for (int n = 3; --n >= 0; el = el->next())
  {
    if (el->data() == Interval({ 1., 5. }))
      REQUIRE(el->child()->child()->child() == 0);
    else
      REQUIRE(el->child() == 0);
  }
  REQUIRE(el == 0);
}

TEST_CASE("containement_basic_002", "[Containement]")
{
  Utils::ContainementTree<Interval> cnt_interv(compare_intervals);
  Interval intervals[] = {
    { 0., 1. },
    { -1., 1. },
    { -2., 2. },
    { 5., 10. },
    { -3., 20. },
    { 11., 13. }
  };
  for (const auto& i : intervals)
    cnt_interv.add(i);
  std::queue<const Utils::ContainementTree<Interval>::Element*> tree_queue;
  tree_queue.push(cnt_interv.root());
  auto nmbr = 0;
  while (!tree_queue.empty())
  {
    const auto el = tree_queue.front();
    tree_queue.pop();
    ++nmbr;
    if (el->child() != nullptr)
    {
      tree_queue.push(el->child());
      REQUIRE(compare_intervals(*el, *el->child()) > 0);
    }
    if (el->next() != nullptr)
    {
      tree_queue.push(el->next());
      REQUIRE(compare_intervals(*el, *el->next()) == 0);
    }
  }
  REQUIRE(nmbr == std::size(intervals));
}