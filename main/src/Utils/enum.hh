

#pragma once

#include <string>

using EnumIntType = unsigned char;

namespace Utils {
void fill(const char* _str, std::string _str_arr[], EnumIntType _n);

template <EnumIntType str_numberT>
struct EnumStrings
{
  EnumStrings(const char* _val)
  {
    Utils::fill(_val, vals_, str_numberT);
  }
  std::string vals_[str_numberT + 1];
};

} // namespace Utils

#define MAKE_ENUM(EnumClass, ...)                               \
enum class EnumClass : EnumIntType { __VA_ARGS__, ENUM_SIZE };  \
struct EnumClass##Helper                                        \
{                                                               \
  static const char* to_string(const EnumClass _e)              \
  { return strings()[static_cast<EnumIntType>(_e)].c_str(); }   \
  static EnumClass to_enum(const char* _str)                    \
  {                                                             \
    for (size_t i = 0;                                          \
         i < static_cast<size_t>(EnumClass::ENUM_SIZE); ++i)    \
      if (strings()[i] == _str)                                 \
        return static_cast<EnumClass>(i);                       \
    return EnumClass::ENUM_SIZE;                                \
  }                                                             \
private:                                                        \
  static const std::string* strings()                           \
  {                                                             \
  static Utils::EnumStrings<EnumIntType(EnumClass::ENUM_SIZE)>  \
     strings(#__VA_ARGS__);                                     \
  return strings.vals_;                                         \
  }                                                             \
};

