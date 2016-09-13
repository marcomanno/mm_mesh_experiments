#include "saver.hh"

#include <functional>
#include <map>
#include <set>

namespace Topo
{

namespace
{

typedef void(*SaveFunction)(std::ostream&, const Object*, ISaver*);

struct SaveMap
{
  SaveMap()
  {
    fill_save_map<int(SubType::ENUM_SIZE) - 1>();
  }
  SaveFunction operator[](SubType _sb)
  {
    return save_functs_[_sb];
  }

private:
  typedef std::map<SubType, SaveFunction> SaveMapInt;
  SaveMapInt save_functs_;

  template <int _sb> void fill_save_map()
  {
    SaveMapInt::value_type x(SubType(_sb), object_saver<SubType(_sb)>);
    save_functs_.insert(x);
    fill_save_map<_sb - 1>();
  }
  template <> void fill_save_map<-1>() {}
};
}// namespace

struct Saver : public ISaver
{
  Saver(std::ostream* _str) : str_(_str) {}
  void save(const Object* _obj) override;
private:
  std::ostream* str_;
  std::vector<Object*> objs_;
  std::set<Identifier> ids_;
};

std::shared_ptr<ISaver> ISaver::make(std::ostream& _str)
{
  return std::make_shared<Saver>(&_str);
}

void Saver::save(const Object* _obj)
{
  const auto id = _obj->id();
  *str_ << id;
  auto it = ids_.insert(id);
  if (!it.second)
    return;
  static SaveMap sm;
  (*sm[_obj->sub_type()])(*str_, _obj, this);
}

}//banespace Topo
