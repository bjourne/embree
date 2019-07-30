#pragma once

template<int M, int K>
static __forceinline bool
intersectKRaysMTris(Vec3vf<K> o, Vec3vf<K> d,
                    vfloat<K> tn, vfloat<K> tf,
                    size_t i,
                    const vbool<K>& valid0,
                    MTHit<K>& hit,
                    const TriangleM<M>& tri)
{
  Vec3vf<K> v0 = broadcast<vfloat<K>>(tri.v0, i);
  Vec3vf<K> e1 = broadcast<vfloat<K>>(tri.e1, i);
  Vec3vf<K> e2 = broadcast<vfloat<K>>(tri.e2, i);

  Vec3vf<K> v1 = v0 - e1;
  Vec3vf<K> v2 = e2 + v0;

  Vec3vf<K> v0o = v0 - o;
  Vec3vf<K> v1o = v1 - o;
  Vec3vf<K> v2o = v2 - o;

  vfloat<K> w2 = dot(d, cross(v1o, v0o));
  vfloat<K> w0 = dot(d, cross(v2o, v1o));
  vbool<K> s2 = w2 >= vfloat<K>(zero);
  vbool<K> s0 = w0 >= vfloat<K>(zero);

  vfloat<K> w1 = dot(d, cross(v0o, v2o));
  vbool<K> s1 = w1 >= vfloat<K>(zero);
  vbool<K> valid = (s2 == s1) & (s0 == s2);

  Vec3vf<K> ng = cross(e2, e1);
  vfloat<K> den = dot(ng, d);
  vfloat<K> t = dot(ng, v0o) * rcp(den);
  valid &= (tn < t) & (t <= tf);
  if (likely(none(valid))) {
    return false;
  }

  vfloat<K> rcpSum = rcp(w0 + w1 + w2);
  vfloat<K> u = w1 * rcpSum;
  vfloat<K> v = w2 * rcpSum;

  new (&hit) MTHit<K>(valid, u, v, t, ng);
  return true;
}

template<int M>
static __forceinline bool
intersect1RayMTris(const Vec3vf<M>& o, Vec3vf<M> d,
                   vfloat<M> tn, vfloat<M> tf,
                   const TriangleM<M>& tri, MTHit<M>& hit)
{
  Vec3vf<M> v1 = tri.v0 - tri.e1;
  Vec3vf<M> v2 = tri.e2 + tri.v0;

  Vec3vf<M> v0o = tri.v0 - o;
  Vec3vf<M> v1o = v1 - o;
  Vec3vf<M> v2o = v2 - o;

  vfloat<M> w2 = dot(d, cross(v1o, v0o));
  vfloat<M> w0 = dot(d, cross(v2o, v1o));
  vbool<M> s2 = w2 >= vfloat<M>(zero);
  vbool<M> s0 = w0 >= vfloat<M>(zero);

  vfloat<M> w1 = dot(d, cross(v0o, v2o));
  vbool<M> s1 = w1 >= vfloat<M>(zero);
  vbool<M> valid = (s2 == s1) & (s0 == s2);

  Vec3vf<M> ng = cross(tri.e2, tri.e1);
  vfloat<M> den = dot(ng, d);
  vfloat<M> t = dot(ng, v0o) * rcp(den);
  valid &= (tn < t) & (t <= tf);
  if (likely(none(valid))) {
    return false;
  }

  vfloat<M> rcpSum = rcp(w0 + w1 + w2);
  vfloat<M> u = w1 * rcpSum;
  vfloat<M> v = w2 * rcpSum;

  new (&hit) MTHit<M>(valid, u, v, t, ng);
  return true;

}
