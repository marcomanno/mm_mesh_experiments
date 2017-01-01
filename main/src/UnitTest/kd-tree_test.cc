#include "Catch/catch.hpp"
#include "Geo/kd-tree.hh"

#include <list>

struct KdTreeElement
{
  static const size_t DIM = 3;
  Geo::Vector3 pt_ =
  {
    double(std::rand()) / RAND_MAX,
    double(std::rand()) / RAND_MAX,
    double(std::rand()) / RAND_MAX
  };
  Geo::Range<3> box_;
  const Geo::Vector3& point() const { return pt_; }
  const Geo::Range<3> box() const
  {
    Geo::Range<3> box;
    box.extr_[0] = box.extr_[1] = pt_;
    box.fatten(0.1);
    return box;
  }
};

void print_box(const Geo::Range<3>& _box)
{
  std::cout << _box.extr_[0] << "--" << _box.extr_[1] << "\n";
}

TEST_CASE("Kd-Tree1", "[KDTREE]")
{
  for (int iii = 90; iii < 130; ++iii)
    for (int jjj = 80; jjj < 100; ++jjj)
    {

      std::vector<KdTreeElement> kk0(91);
      Geo::KdTree<KdTreeElement> kdt0;
      kdt0.insert(kk0.begin(), kk0.end());
      kdt0.compute();

      std::vector<KdTreeElement> kk1(71);
      Geo::KdTree<KdTreeElement> kdt1;
      kdt1.insert(kk1.begin(), kk1.end());
      kdt1.compute();
      if ((kdt0.box() * kdt1.box()).empty())
        return;

      auto coll_pairs = find_kdtree_couples(kdt0, kdt1);
      std::sort(coll_pairs.begin(), coll_pairs.end());

      std::vector<std::array<size_t, 2>> coll_pairs1;
      for (size_t i = 0; i < kk0.size(); ++i)
        for (size_t j = 0; j < kk1.size(); ++j)
          if (!(kdt0[i].box() * kdt1[j].box()).empty())
            coll_pairs1.emplace_back(std::array<size_t, 2>{ i, j });

      REQUIRE(coll_pairs1.size() == coll_pairs.size());
      size_t ii[2] = { 0, 0 };
      while (ii[0] < coll_pairs.size() && ii[1] < coll_pairs1.size())
      {
        if (coll_pairs[ii[0]] < coll_pairs1[ii[1]])
          std::cout << "- pair 0 - " << coll_pairs[ii[0]++] << "\n";
        else if (coll_pairs[ii[0]] > coll_pairs1[ii[1]])
          std::cout << "- pair 1 - " << coll_pairs1[ii[1]++] << "\n";
        else
        {
          ++ii[0]; ++ii[1];
        }
      }
      for (size_t j = 0; j < 2; ++j)
      {
        while (ii[j] < coll_pairs[j].size())
          std::cout << "- pair " << j << " - " << coll_pairs[j][ii[j]++] << "\n";
      }

#if 0
      std::function<void(size_t)> process_kt_element =
        [&kdt0, &process_kt_element](size_t _i)
      {
        std::cout << "=========== " << _i << std::endl;
        for (size_t j = 0; j < 2; ++j)
        {
          if (kdt0.box(_i, j).empty())
            continue;
          bool leaf;
          auto child_idx = kdt0.child(_i, 0, leaf);
          if (!leaf)
            process_kt_element(child_idx);
          else
          {
            size_t st, en;
            if (!kdt0.leaf_range(_i, st, en))
              return;
            std::cout << st << " " << en << "\n";
          }
        }
      };
      process_kt_element(0);
#endif
    }
}
