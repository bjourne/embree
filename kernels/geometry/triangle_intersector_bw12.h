#pragma once

template<int K>
static __forceinline bool
isectAlgo(const Vec3vf<K>& o, const Vec3vf<K>& d,
          const vfloat<K>& tn, const vfloat<K>& tf,
          const Vec3vf<K>& n0, const Vec3vf<K>& n1, const Vec3vf<K>& n2,
          const vfloat<K>& d0, const vfloat<K>& d1, const vfloat<K>& d2,
          const Vec3vf<K>& ng,
          MTHit<K>& hit, const vbool<K>& valid0) {
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
  Vec3vf<K> n0 = broadcast<vfloat<K>>(tri.n0, i);
  Vec3vf<K> n1 = broadcast<vfloat<K>>(tri.n1, i);
  Vec3vf<K> n2 = broadcast<vfloat<K>>(tri.n2, i);
  Vec3vf<K> ng = broadcast<vfloat<K>>(tri.ng, i);

  vfloat<K> d0 = vfloat<K>(tri.d0[i]);
  vfloat<K> d1 = vfloat<K>(tri.d1[i]);
  vfloat<K> d2 = vfloat<K>(tri.d2[i]);

  return isectAlgo<K>(o, d,
                      tn, tf,
                      n0, n1, n2,
                      d0, d1, d2,
                      ng,
                      hit, valid0);
}

/*! Intersect 1 ray with one of M triangles. */
template<int M>
static __forceinline bool
intersect1RayMTris(Vec3vf<M> o, Vec3vf<M> d,
                   vfloat<M> tn, vfloat<M> tf,
                   const TriangleM<M>& tri, MTHit<M>& hit)
{
  return isectAlgo<M>(o, d, tn, tf,
                      tri.n0, tri.n1, tri.n2,
                      tri.d0, tri.d1, tri.d2,
                      tri.ng,
                      hit, true);
}
