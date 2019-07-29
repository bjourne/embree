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
  const vfloat<K> U = dot(e2, R) ^ sgnDen;
  valid &= U >= 0.0f;
  if (likely(none(valid)))
    return false;

  /* test against edge v0 e1 */
  const vfloat<K> V = dot(e1, R) ^ sgnDen;
  valid &= V >= 0.0f;
  if (likely(none(valid)))
    return false;

  /* test against edge e1 e2 */
  const vfloat<K> W = absDen-U-V;
  valid &= W >= 0.0f;
  if (likely(none(valid)))
    return false;

  /* perform depth test */
  const vfloat<K> T = dot(tri_Ng,C) ^ sgnDen;
  valid &= (absDen*ray.tnear() < T) & (T <= absDen*ray.tfar);
  if (unlikely(none(valid)))
    return false;

  valid &= den != vfloat<K>(zero);
  if (unlikely(none(valid)))
    return false;

  /* calculate hit information */
  const vfloat<K> rcpAbsDen = rcp(absDen);
  new (&hit) MTHitK<K>(valid,
                       U * rcpAbsDen,
                       V * rcpAbsDen,
                       T * rcpAbsDen,
                       tri_Ng);
  return true;
}

template<int M, int K>
static __forceinline bool
intersectKthRayMTris(const RayK<K>& ray,
                     size_t k,
                     const TriangleM<M>& tri,
                     MTHitM<M>& hit)
{
  const Vec3vf<M> tri_Ng = cross(tri.e2, tri.e1);

  const Vec3vf<M> O = broadcast<vfloat<M>>(ray.org, k);
  const Vec3vf<M> D = broadcast<vfloat<M>>(ray.dir, k);
  const Vec3vf<M> C = Vec3vf<M>(tri.v0) - O;
  const Vec3vf<M> R = cross(C,D);
  const vfloat<M> den = dot(Vec3vf<M>(tri_Ng),D);
  const vfloat<M> absDen = abs(den);
  const vfloat<M> sgnDen = signmsk(den);

  /* perform edge tests */
  const vfloat<M> U = dot(Vec3vf<M>(tri.e2), R) ^ sgnDen;
  const vfloat<M> V = dot(Vec3vf<M>(tri.e1), R) ^ sgnDen;

  vbool<M> valid = (den != vfloat<M>(zero)) &
    (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
  if (likely(none(valid)))
    return false;

  /* perform depth test */
  const vfloat<M> T = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
  valid &= (absDen * vfloat<M>(ray.tnear()[k]) < T) &
    (T <= absDen * vfloat<M>(ray.tfar[k]));
  if (likely(none(valid)))
    return false;

  /* calculate hit information */
  const vfloat<M> rcpAbsDen = rcp(absDen);
  new (&hit) MTHitM<M>(valid,
                       U * rcpAbsDen,
                       V * rcpAbsDen,
                       T * rcpAbsDen,
                       tri_Ng);
  return true;
}

/*! Intersect 1 ray with one of M triangles. */
template<int M>
static __forceinline bool
intersect1RayMTris(Ray& ray,
                   const TriangleM<M>& tri,
                   MTHitM<M>& hit)
{
  vbool<M> valid = true;
  const Vec3vf<M> o = Vec3vf<M>(ray.org);
  const Vec3vf<M> d = Vec3vf<M>(ray.dir);
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
  valid &= (den != vfloat<M>(zero)) &
    (u >= 0.0f) & (v >= 0.0f) & (u+v<=absDen);
  if (likely(none(valid)))
    return false;

  /* perform depth test */
  const vfloat<M> t = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
  valid &= (absDen*vfloat<M>(ray.tnear()) < t) &
    (t <= absDen*vfloat<M>(ray.tfar));
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


// Crown 9.33
