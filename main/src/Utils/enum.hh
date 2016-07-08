

#pragma once

#include <bitset>

#define MAKE_ENUM(NAME, ...) enum struct NAME { __VA_ARGS__, ENUM_SIZE };\
struct NAME##Mask                                                        \
{                                                                        \
  static constexpr size_t size() { return size_t(NAME::ENUM_SIZE); }     \
  static NAME get(size_t _i) { return NAME(_i); }                        \
  static size_t get(NAME _n) { return size_t(_n); }                      \
  bool on(NAME _n) const { return mask_[get(_n)]; }                      \
  bool on(size_t _i) const { return mask_[_i]; }                         \
  void set(NAME _n) { mask_[get(_n)] = true; }                           \
  void set(size_t _i) { mask_[_i] = true; }                              \
  void reset(NAME _n) { mask_[get(_n)] = false; }                        \
  void reset(size_t _i) { mask_[_i] = false; }                           \
private:                                                                 \
  std::bitset<size_t(NAME::ENUM_SIZE)> mask_;                            \
};

#if 0

#include <bitset>
#include <string>

#define EXPAND(X) X

#define MAKE_STRING_1(str     ) #str
#define MAKE_STRING_2(str, ...) #str, MAKE_STRING_1(__VA_ARGS__)
#define MAKE_STRING_3(str, ...) #str, MAKE_STRING_2(__VA_ARGS__)
#define MAKE_STRING_4(str, ...) #str, MAKE_STRING_3(__VA_ARGS__)
#define MAKE_STRING_5(str, ...) #str, MAKE_STRING_4(__VA_ARGS__)
#define MAKE_STRING_6(str, ...) #str, MAKE_STRING_5(__VA_ARGS__)
#define MAKE_STRING_7(str, ...) #str, MAKE_STRING_6(__VA_ARGS__)
#define MAKE_STRING_8(str, ...) #str, MAKE_STRING_7(__VA_ARGS__)

#define PRIMITIVE_CAT(a, b) a##b
#define MAKE_STRING_(N, ...) PRIMITIVE_CAT(MAKE_STRING_, N) (__VA_ARGS__)
#define MAKE_STRING(...) MAKE_STRING_(PP_NARG(__VA_ARGS__), __VA_ARGS__)

#define PP_RSEQ_N() 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
#define PP_ARG_N(_1,_2,_3,_4,_5,_6,_7,_8,_9, _10,_N,...) _N
#define PP_NARG_(...) EXPAND(PP_ARG_N(__VA_ARGS__))
#define PP_NARG(...) EXPAND(PP_NARG_(__VA_ARGS__, PP_RSEQ_N()))

#define MAKE_ENUM(NAME, ...)                                                    \
  enum struct NAME { __VA_ARGS__ };                                             \
  struct NAME##Helper                                                           \
  {                                                                             \
    static constexpr unsigned N() { return PP_NARG(__VA_ARGS__); }              \
    static const char * str(const NAME et)                                      \
    {                                                                           \
      static const char* NAME##Str[] = { MAKE_STRING(__VA_ARGS__) };            \
      return NAME##Str[unsigned(et)];                                           \
    }                                                                           \
    static const char * str(const unsigned _i) { return str(NAME(_i % N())); }  \
    static NAME get(unsigned _i) { return NAME(_i % N()); }                     \
    typedef std::bitset<PP_NARG(__VA_ARGS__)> MaskBits;                         \
    struct Mask                                                                 \
    {                                                                           \
      Mask() {}                                                                 \
      Mask(const MaskBits& _msk_bit) : mask_(_msk_bit) {}                       \
      Mask(const NAME _e) : Mask(unsigned(_e)) { }                              \
      Mask(const unsigned _i) { mask_[_i] = true; }                             \
      Mask(const Mask& _me) : mask_(_me.mask_) {}                               \
      Mask& operator +=(const Mask& _me) { mask_ |= _me.mask_; return *this; }  \
      Mask& operator -=(const Mask& _me) { mask_ &= ~_me.mask_; return *this; } \
      Mask& operator *=(const Mask& _me) { mask_ &= _me.mask_; return *this; }  \
      bool on() const { return mask_.any(); }                                   \
      bool on(const NAME e) const { return mask_[unsigned(e)]; }                \
      bool on(unsigned _i) const { return mask_[_i % N()]; }                    \
      const MaskBits& mask_bits() { return mask_; }                             \
    private:                                                                    \
      MaskBits mask_;                                                           \
    };                                                                          \
  };

#define GENERIC_OPERATOR(OP)                                            \
template <class ObjT> ObjT operator OP (const ObjT& _a, const ObjT& _b) \
{ return (ObjT(_a) OP##= _b); }

GENERIC_OPERATOR(+)
GENERIC_OPERATOR(-)
GENERIC_OPERATOR(*)
#undef GENERIC_OPERATOR

#endif