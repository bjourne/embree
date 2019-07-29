#pragma once

template<int M, int K>
static __forceinline bool
intersectKRaysMTris(const RayK<K>& ray,
                    size_t i,
                    const vbool<K>& valid0,
                    MTHitK<K>& hit,
                    const TriangleM<M>& tri)
{
  // Do broadcasting and cross prod.
  Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0, i);
  Vec3<vfloat<K>> e1 = broadcast<vfloat<K>>(tri.e1, i);
  Vec3<vfloat<K>> e2 = broadcast<vfloat<K>>(tri.e2, i);
  Vec3<vfloat<K>> tri_Ng = cross(e2, e1);

  /* calculate denominator */
  vbool<K> valid = valid0;
  const Vec3vf<K> C = v0 - ray.org;
  const Vec3vf<K> R = cross(C, ray.dir);
  const vfloat<K> den = dot(tri_Ng, ray.dir);
  const vfloat<K> absDen = abs(den);
  const vfloat<K> sgnDen = signmsk(den);

  /* test against edge e2 v0 */
  const vfloat<K> u = dot(e2, R) ^ sgnDen;
  valid &= u >= 0.0f;
  if (likely(none(valid))) {
    return false;
  }

  /* test against edge v0 e1 */
  const vfloat<K> v = dot(e1, R) ^ sgnDen;
  valid &= v >= 0.0f;
  if (likely(none(valid))) {
    return false;
  }

  /* test against edge e1 e2 */
  const vfloat<K> w = absDen-u-v;
  valid &= w >= 0.0f;
  if (likely(none(valid))) {
    return false;
  }

  /* perform depth test */
  const vfloat<K> t = dot(tri_Ng,C) ^ sgnDen;
  valid &= (absDen*ray.tnear() < t) & (t <= absDen*ray.tfar);
  if (unlikely(none(valid))) {
    return false;
  }

  valid &= den != vfloat<K>(zero);
  if (unlikely(none(valid))) {
    return false;
  }

  /* calculate hit information */
  const vfloat<K> rcpAbsDen = rcp(absDen);
  new (&hit) MTHitK<K>(valid,
                       u * rcpAbsDen,
                       v * rcpAbsDen,
                       t * rcpAbsDen,
                       tri_Ng);
  return true;
}

/*! Intersect 1 ray with one of M triangles. */
template<int M>
static __forceinline bool
intersect1RayMTris(Vec3vf<M> o, Vec3vf<M> d,
                   vfloat<M> tnear, vfloat<M> tfar,
                   const TriangleM<M>& tri, MTHitM<M>& hit)
{
  const Vec3vf<M> C = Vec3vf<M>(tri.v0) - o;
  const Vec3vf<M> R = cross(C,d);
  const Vec3vf<M> tri_Ng = cross(tri.e2, tri.e1);
  const vfloat<M> den = dot(Vec3vf<M>(tri_Ng), d);
  const vfloat<M> absDen = abs(den);
  const vfloat<M> sgnDen = signmsk(den);

  /* perform edge tests */
  const vfloat<M> u = dot(R, Vec3vf<M>(tri.e2)) ^ sgnDen;
  const vfloat<M> v = dot(R, Vec3vf<M>(tri.e1)) ^ sgnDen;

  /* perform backface culling */
  vbool<M> valid = (den != vfloat<M>(zero)) &
    (u >= 0.0f) & (v >= 0.0f) & (u+v<=absDen);
  if (likely(none(valid)))
    return false;

  /* perform depth test */
  const vfloat<M> t = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
  valid &= ((absDen * tnear) < t) & (t <= absDen * tfar);
  if (likely(none(valid)))
    return false;

  /* update hit information */
  const vfloat<M> rcpAbsDen = rcp(absDen);
  new (&hit) MTHitM<M>(valid,
                       u * rcpAbsDen,
                       v * rcpAbsDen,
                       t * rcpAbsDen,
                       tri_Ng);
  return true;
}


// Crown 9.3 viewer
// Crown 12.9 viewer_ispc
