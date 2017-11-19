

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
enum class EnumClass : unsigned int { __VA_ARGS__, ENUM_SIZE }; \
struct EnumClass##_to_string                                    \
{                                                               \
  static const char* get(const EnumClass _e)                    \
  {                                                             \
  static Utils::EnumStrings<EnumIntType(EnumClass::ENUM_SIZE)>  \
     strings(#__VA_ARGS__);                                     \
  return strings.vals_[EnumIntType(_e)].c_str();                \
  }                                                             \
};
