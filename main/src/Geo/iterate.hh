
#pragma once

namespace Geo
{
template <size_t N> struct iterate_forw
{
  template <class operation> static auto eval(const operation & op)
  {
    iterate_forw < N - 1 > ::eval(op);
    return op(N - 1);
  }
} ;
template <> struct iterate_forw<0>
{
  template <class operation> static auto eval(const operation &) { }
} ;

template <size_t N> struct iterate_back
{
  template <class operation> static auto eval(const operation & op)
  {
    iterate_back < N - 1 > ::eval(op);
    return op(N - 1);
  }
} ;
template <> struct iterate_back<0>
{
  template <class operation> static auto eval(const operation & op) { }
} ;

template <size_t N, bool targ = true> struct iterate_forw_until
{
  template <class operation> static auto eval(const operation & op)
  {
    if (iterate_forw_until < N - 1, targ > ::eval(op) != targ)
      return !targ;
    return op(N - 1);
  }
} ;
template < bool targ > struct iterate_forw_until<0, targ>
{
  template <class operation> static bool eval(const operation & op) { return targ; }
} ;

template <size_t N, bool targ = true> struct iterate_back_until
{
  template <class operation> static auto eval(const operation & op)
  {
    if (op(N - 1) != targ)
      return !targ;
    return iterate_forw_until < N - 1, targ > ::eval(op);
  }
} ;
template < bool targ > struct iterate_back_until<0, targ>
{
  template <class operation> static bool eval(const operation & op) { return targ; }
} ;
}//Geo
