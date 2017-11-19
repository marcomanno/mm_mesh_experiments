#include "enum.hh"

namespace Utils {

void fill(const char* _str, std::string _str_arr[], EnumIntType _n)
{
  const char delimiter[] = ", ";
  const char* p = _str;
  for (EnumIntType i = 0; i < _n; ++i)
  {
    auto p1 = strstr(p, delimiter);
    if (p1 == nullptr)
    {
      _str_arr[i] = p;
      return;
    }
    _str_arr[i] = std::string(p, 0, p1 - p);
    p = p1 + std::size(delimiter) - 1;
  }
}

} // namespace Utils
