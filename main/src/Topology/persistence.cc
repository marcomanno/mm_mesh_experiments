#include "persistence.hh"
#include "Utils/bindata.hh"

#include <functional>
#include <map>
#include <set>

namespace Topo
{

namespace
{

typedef void (*SaveFunction)(std::ostream&, const Object*, ISaver*);
typedef WrapObject (*LoadFunction)(std::istream&, ILoader*);

struct PersistenceFunctions
{
  SaveFunction _sav_fun;
  LoadFunction _load_fun;
};

struct PersistenceMap
{
  PersistenceMap()
  {
    fill_persistence_map<int(SubType::ENUM_SIZE) - 1>();
  }
  const PersistenceFunctions& operator[](SubType _sb)
  {
    return save_functs_[_sb];
  }

private:
  typedef std::map<SubType, PersistenceFunctions> SaveMapInt;
  SaveMapInt save_functs_;

  template <int _sb> void fill_persistence_map()
  {
    auto& pers_funs = save_functs_[SubType(_sb)];
    pers_funs._load_fun = object_loader<SubType(_sb)>;
    pers_funs._sav_fun = object_saver<SubType(_sb)>;
    fill_persistence_map<_sb - 1>();
  }
  template <> void fill_persistence_map<-1>() {}
};

static PersistenceMap pers_map__;

}// namespace

struct Saver : public ISaver
{
  Saver(std::ostream* _str) : str_(_str) {}
  void save(const Object* _obj) override;
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
  if (!saved_objs_.insert(_obj).second)
    return;
  *str_ << Utils::BinData<size_t>(size_t(_obj->sub_type()));
  pers_map__[_obj->sub_type()]._sav_fun(*str_, _obj, this);
}

struct Loader : public ILoader
{
  Loader(std::istream* _str) : str_(_str) {}
  virtual WrapObject load() override;
private:
  std::istream* str_;
  std::map<size_t, WrapObject> loaded_objs_;
};

std::shared_ptr<ILoader> ILoader::make(std::istream& _str)
{
  return std::make_shared<Loader>(&_str);
}

WrapObject Loader::load()
{
  size_t id;
  *str_ >> Utils::BinData<size_t>(id);
  auto it = loaded_objs_.emplace(id, nullptr);
  if (it.second)
  {
    size_t sub_ty;
    *str_ >> Utils::BinData<size_t>(sub_ty);
    it.first->second =
      pers_map__[Topo::SubType(sub_ty)]._load_fun(*str_, this);
  }
  return it.first->second;
}

}//banespace Topo
