#pragma once

template<int K>
static __forceinline bool
isectAlgo(const Vec3vf<K>& o, const Vec3vf<K>& d,
          const vfloat<K>& tn, const vfloat<K>& tf,
          const vfloat<K>& aa, const vfloat<K>& bb, const vfloat<K>& cc,
          const vfloat<K>& dd, const vfloat<K>& ee, const vfloat<K>& ff,
          const vfloat<K>& gg, const vfloat<K>& hh, const vfloat<K>& ii,
          int ci,
          const Vec3vf<K> ng,
          MTHit<K>& hit, const vbool<K>& valid0) {

  vfloat<K> u, v, t;
  if (ci == 0) {
    vfloat<K> t_o = o.x + gg * o.y + hh * o.z + ii;
    vfloat<K> t_d = d.x + gg * d.y + hh * d.z;
    t = -t_o * rcp(t_d);
    Vec3vf<K> wr = o + d * t;
    u = aa * wr.y + bb * wr.z + cc;
    v = dd * wr.y + ee * wr.z + ff;
  } else if (ci == 1) {
    vfloat<K> t_o = gg * o.x + o.y + hh * o.z + ii;
    vfloat<K> t_d = gg * d.x + d.y + hh * d.z;
    t = -t_o * rcp(t_d);
    Vec3vf<K> wr = o + d * t;
    u = aa * wr.x + bb * wr.z + cc;
    v = dd * wr.x + ee * wr.z + ff;
  } else {
    vfloat<K> t_o = o.x * gg + o.y * hh + o.z + ii;
    vfloat<K> t_d = d.x * gg + d.y * hh + d.z;
    t = -t_o * rcp(t_d);
    Vec3vf<K> wr = o + d * t;
    u = aa * wr.x + bb * wr.y + cc;
    v = dd * wr.x + ee * wr.y + ff;
  }
  const vbool<K> valid = valid0
    & (u >= 0.0f) & (v >= 0.0f) & (u + v <= 1.0f)
    & (tn < t) & (t <= tf);
  if (likely(none(valid)))
    return false;
  new (&hit) MTHit<K>(valid, u, v, t, ng);
  return true;
}

template<int M, int K>
static __forceinline bool
intersectKRaysMTris(const Vec3vf<K>& o, const Vec3vf<K>& d,
                    const vfloat<K>& tn, const vfloat<K>& tf,
                    size_t i,
                    const vbool<K>& valid0,
                    MTHit<K>& hit,
                    const TriangleM<M>& tri)
{
  vfloat<K> aa = vfloat<K>(tri.aa[i]);
  vfloat<K> bb = vfloat<K>(tri.bb[i]);
  vfloat<K> cc = vfloat<K>(tri.cc[i]);
  vfloat<K> dd = vfloat<K>(tri.dd[i]);
  vfloat<K> ee = vfloat<K>(tri.ee[i]);
  vfloat<K> ff = vfloat<K>(tri.ff[i]);
  vfloat<K> gg = vfloat<K>(tri.gg[i]);
  vfloat<K> hh = vfloat<K>(tri.hh[i]);
  vfloat<K> ii = vfloat<K>(tri.ii[i]);
  return isectAlgo<K>(o, d,
                      tn, tf,
                      aa, bb, cc,
                      dd, ee, ff,
                      gg, hh, ii,
                      tri.ci[i], tri.ng,
                      hit, valid0);
}

/*! Intersect 1 ray with one of M triangles. */
template<int M>
static __forceinline bool
intersect1RayMTris(Vec3vf<M> o, Vec3vf<M> d,
                   vfloat<M> tn, vfloat<M> tf,
                   const TriangleM<M>& tri, MTHit<M>& hit)
{
  printf("Shouldn't be here!!!\n");
  assert(false);
  return false;
}
