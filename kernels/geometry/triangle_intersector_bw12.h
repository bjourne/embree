#pragma once

// 14.6 for crown
template<int M, int K>
static __forceinline bool
intersectKRaysMTris(const Vec3vf<K>& o, const Vec3vf<K>& d,
                    const vfloat<K>& tn, const vfloat<K>& tf,
                    size_t i,
                    const vbool<K>& valid0,
                    MTHit<K>& hit,
                    const TriangleM<M>& tri)
{
  Vec3vf<K> n0 = broadcast<vfloat<K>>(tri.n0, i);
  Vec3vf<K> n1 = broadcast<vfloat<K>>(tri.n1, i);
  Vec3vf<K> n2 = broadcast<vfloat<K>>(tri.n2, i);

  vfloat<K> d0 = vfloat<K>(tri.d0[i]);
  vfloat<K> d1 = vfloat<K>(tri.d1[i]);
  vfloat<K> d2 = vfloat<K>(tri.d2[i]);

  vfloat<K> t_o = dot(o, n2) + d2;
  vfloat<K> t_d = dot(d, n2);
  vfloat<K> t = -t_o * rcp(t_d);
  Vec3vf<K> wr = o + d * t;
  vfloat<K> u = dot(wr, n0) + d0;
  vfloat<K> v = dot(wr, n1) + d1;
  const vbool<K> valid = valid0
    & (u >= 0.0f) & (v >= 0.0f) & (u + v <= 1.0f)
    & (tn < t) & (t <= tf);
  if (likely(none(valid)))
    return false;

  new (&hit) MTHit<K>(valid, u, v, t,
                      broadcast<vfloat<K>>(tri.ng, i));
  return true;
}

// 7.05 for crown
template<int M>
static __forceinline bool
intersect1RayMTris(const Vec3vf<M>& o, const Vec3vf<M>& d,
                   const vfloat<M>& tn, const vfloat<M>& tf,
                   const TriangleM<M>& tri, MTHit<M>& hit)
{

  vfloat<M> t_o = dot(o, tri.n2) + tri.d2;
  vfloat<M> t_d = dot(d, tri.n2);
  vfloat<M> t = -t_o * rcp(t_d);
  Vec3vf<M> wr = o + d * t;
  vfloat<M> u = dot(wr, tri.n0) + tri.d0;
  vfloat<M> v = dot(wr, tri.n1) + tri.d1;
  const vbool<M> valid = (u >= 0.0f) & (v >= 0.0f)
    & (u + v <= 1.0f) & (tn < t) & (t <= tf);
  if (likely(none(valid)))
    return false;
  new (&hit) MTHit<M>(valid, u, v, t, tri.ng);
  return true;
}
