#pragma once

template<int M, int K>
static __forceinline bool
intersectKRaysMTris(const RayK<K>& ray,
                    size_t i,
                    const vbool<K>& valid0,
                    MTHitK<K>& hit,
                    const TriangleM<M>& tri)
{
  Vec3vf<K> n0 = broadcast<vfloat<K>>(tri.n0, i);
  Vec3vf<K> n1 = broadcast<vfloat<K>>(tri.n1, i);
  Vec3vf<K> n2 = broadcast<vfloat<K>>(tri.n2, i);

  vfloat<K> d0 = vfloat<K>(tri.d0[i]);
  vfloat<K> d1 = vfloat<K>(tri.d1[i]);
  vfloat<K> d2 = vfloat<K>(tri.d2[i]);

  vfloat<K> det = dot(n0, ray.dir);
  vfloat<K> dett = d0 - dot(ray.org, n0);
  Vec3vf<K> wr = ray.org * det + ray.dir * dett;

  vfloat<K> u = dot(wr, n1) + det * d1;
  vfloat<K> v = dot(wr, n2) + det * d2;
  vfloat<K> tmpdet0 = det - u - v;
  vfloat<K> pdet0 = (tmpdet0 ^ u) | (u ^ v);
  vfloat<K> msk = signmsk(pdet0);

  // This allows early return without hitting the division.
  vbool<K> valid = valid0 & asInt(msk) == vint<K>(zero);
  if (likely(none(valid))) {
    return false;
  }
  vfloat<K> rdet = rcp(det);
  u = u * rdet;
  v = v * rdet;
  vfloat<K> t = dett * rdet;
  valid &= (ray.tnear() < t) & (t <= ray.tfar);
  if (unlikely(none(valid))) {
    return false;
  }
  new (&hit) MTHitK<K>(valid, u, v, t, n0);
  return true;
}

/*! Intersect 1 ray with one of M triangles. */
template<int M>
static __forceinline bool
intersect1RayMTris(Vec3vf<M> o, Vec3vf<M> d,
                   vfloat<M> tnear, vfloat<M> tfar,
                   const TriangleM<M>& tri, MTHitM<M>& hit)
{
  vfloat<M> det = dot(tri.n0, d);
  vfloat<M> dett = tri.d0 - dot(o, tri.n0);
  Vec3vf<M> wr = o * det + d * dett;

  vfloat<M> u = dot(wr, tri.n1) + det * tri.d1;
  vfloat<M> v = dot(wr, tri.n2) + det * tri.d2;
  vfloat<M> tmpdet0 = det - u - v;
  vfloat<M> pdet0 = (tmpdet0 ^ u) | (u ^ v);
  vfloat<M> msk = signmsk(pdet0);
  vbool<M> valid = asInt(msk) == vint<M>(zero);
  if (likely(none(valid))) {
    return false;
  }

  vfloat<M> rdet = rcp(det);
  u = u * rdet;
  v = v * rdet;
  vfloat<M> t = dett * rdet;
  valid &= (tnear < t) & (t <= tfar);
  if (likely(none(valid)))
    return false;

  new (&hit) MTHitM<M>(valid, u, v, t, tri.n0);
  return true;
}
