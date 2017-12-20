#pragma once
#include "combinations.hh"

#include <algorithm>
#include <vector>

namespace Geo {

template <typename Par, typename Pt, typename Res = Pt, typename TPar = Par, bool comp_all = true >
void NUBS_eval(const TPar & t, size_t deg, const Par * kn1,
               const Pt * pts, Res * result, int delta = 1)
{
  if (deg == 0)
  {
    new(result) Res(pts[0]);
    // std::cout << "0 [" << *result << "]\n";
    return;
  }
  Par t1 = kn1[     0];
  Par t0 = kn1[-delta];
  Res * new_res = result + delta;
  --deg;
  ++delta;
  if (comp_all)
    NUBS_eval<Par, Pt, Res, TPar, true>(t, deg, kn1  , pts,   new_res,     delta);
  NUBS_eval<Par, Pt, Res, TPar, false>   (t, deg, kn1 + 1, pts + 1, new_res + 1, delta);
  new (&result[0]) Res( ( (t1 - t) * new_res[0] + (t - t0) * new_res[1] ) * (1. / (t1 - t0) ) );
  //std::cout << deg+1 << " [" << *result << "]\n";
}

template <typename Par, typename TPar> Par convert(const TPar & pt)
{
  return static_cast<Par> (pt);
}

inline void * zero_ptr()
{
  static void * p;
  return p;
}

template <typename Pt, typename Par, typename Res = Pt> class Nub
{
  enum { MAX_DEG = 25 } ;
  const Par * m_k;
  const Pt  * m_p;
  size_t m_nk, m_np, m_deg;

public:
  Nub() : m_k(0), m_p(0)
  {
    m_nk = m_np = m_deg = 0;
  }
  bool init(const std::vector<Pt> & ctrp, const std::vector<Par> & kn)
  {
    m_nk = kn.size();
    m_np = ctrp.size();
    if (m_nk < m_np)
      return false;
    m_deg = m_nk - m_np + 1;
    std::ptrdiff_t n_span = m_np - m_deg;
    if (n_span < 0)
      return false;
    m_k = kn.data();
    m_p = ctrp.data();
    return true;
  }
  template <class Iter, typename TPar> 
  bool eval(const TPar & _t, Iter _beg, Iter _end, 
    const Par * _tspan = nullptr, bool _right = false) const
  {
    if (m_nk != m_np + m_deg - 1 || m_deg > MAX_DEG || _beg == _end)
      return false;

    const Par * first_span_k1 = m_k + m_deg;
    const Par * last_span_k1  = m_k + m_nk - m_deg;
    const Par span_sel = _tspan ? *_tspan : convert<Par, TPar>(_t);
    const Par * span_k1 = std::lower_bound(first_span_k1, last_span_k1, span_sel);
    if (_right && span_k1 != last_span_k1 && *span_k1 == span_sel)
      ++span_k1;

    const Pt * first_pt = m_p + (span_k1 - m_k) - m_deg;
    char results[sizeof(Res) * comb<2>(MAX_DEG)];
    Res * result = reinterpret_cast<Res*> (results);
    NUBS_eval<Par, Pt, Res, TPar>(_t, m_deg, span_k1, first_pt, result);
    *_beg = result[0];

    double factor = 1;
    int ider = 0;
    Res * coe = result;
    while (++_beg != _end && ++ider <= m_deg)
    {
      coe += ider;
      for (int n = ider; n > 0; --n)
      {
        const Par * span_v = span_k1;
        for (size_t j = 0; j < n; ++j, ++span_v)
          coe[j] = (coe[j + 1] - coe[j]) * (1. / (span_v[0] - span_v[-n]));
      }
      factor *= Par(m_deg - ider + 1);
      *_beg = coe[0] * factor;
    }

    while (_beg != _end) *_beg++ = Res{ 0 };

    //for (auto i = comb(m_deg, 2); --i >= 0; result[i].~Res());
    return true;
  }
};

template <typename Pt, typename Par, typename Res = Pt> class nub_cv
{
  std::vector<Pt>   m_ctrp;
  std::vector<Par>  m_kn;
  Nub<Pt, Par, Res> m_ev;

public:
  template <class PtIt, class ParIt> void init(
    PtIt pt_begin, PtIt pt_end, ParIt par_begin, ParIt par_end)
  {
    m_ctrp.clear();
    m_kn.clear();
    std::copy(pt_begin, pt_end, std::back_inserter<std::vector < Pt >> (m_ctrp));
    std::copy(par_begin, par_end, std::back_inserter<std::vector < Par >> (m_kn));
    m_ev.init(m_ctrp, m_kn);
  }
  const Nub<Pt, Par, Res> & ev()
  {
    return m_ev;
  }
  const std::vector<Pt>  & ctrp() const { return m_ctrp; }
  const std::vector<Par> & knots() const { return m_kn; }
  bool set(size_t i_cp, const Pt & pt)
  {
    if (i_cp >= m_ctrp.size())
      return false;
    m_ctrp[i_cp] = pt;
    m_ev.init(m_ctrp, m_kn);
    return true;
  }
} ;

} // namespace geo