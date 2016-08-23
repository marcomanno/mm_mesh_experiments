#include "saver.hh"

namespace Topo
{
struct Saver : public ISaver
{
  void add_element(Object* _el) override
  {
    objs_.push_back(_el);
  }
  bool save() override;
  std::vector<Object*> objs_;
};

std::shared_ptr<ISaver> ISaver::make()
{
  return std::make_shared<Saver>();
}

bool Saver::save()
{
  std::sort(objs_.begin(), objs_.end(), [](Object* _a, Object*_b) { return *_a < *_b;});
  auto last = std::unique(
    objs_.begin(), objs_.end(), [](Object* _a, Object*_b) { return *_a == *_b;});
  objs_.erase(last, objs_.end());
}

}