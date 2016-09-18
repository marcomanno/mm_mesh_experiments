#include "persistence.hh"
#include "Utils/bindata.hh"

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
  bool already_saved(const Object* _el);
private:
  std::ostream* str_;
  std::set<const Object*> saved_objs_;
};

std::shared_ptr<ISaver> ISaver::make(std::ostream& _str)
{
  return std::make_shared<Saver>(&_str);
}

void Saver::save(const Object* _obj)
{
  *str_ << Utils::BinData<size_t>(_obj->id());
  if (already_saved(_obj))
    return;
  *str_ << Utils::BinData<size_t>(size_t(_obj->sub_type()));
  static SaveMap sm;
  (*sm[_obj->sub_type()])(*str_, _obj, this);
}

bool Saver::already_saved(const Object* _el)
{
  auto ins = saved_objs_.insert(_el);
  return ins.second;
}

struct Loader : public ILoader
{
  Loader(std::istream* _str) : str_(_str) {}
  virtual Object* load() override;
private:
  std::istream* str_;
  std::map<size_t, Object*> loaded_objs_;
};

std::shared_ptr<ILoader> ILoader::make(std::istream& _str)
{
  return std::make_shared<Loader>(&_str);
}

Object* Loader::load()
{
  size_t id;
  *str_ >> Utils::BinData<size_t>(id);
  auto it = loaded_objs_.emplace(id, nullptr);
  if (it.second)
  {
    it.first->second =
      object_loader<SubType::VERTEX>(*str_, this);
  }
  return it.first->second;
}

}//banespace Topo
