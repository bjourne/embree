#pragma once

// Fix this - doesn't work yet.
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

  vfloat<K> det = dot(n0, ray.org);
  vfloat<K> dett = d0 - dot(ray.org, n0);
  Vec3vf<K> wr = ray.org * det + ray.dir * dett;

  vfloat<K> u = dot(wr, n1) + det * d1;
  vfloat<K> v = dot(wr, n2) + det * d2;
  vfloat<K> tmpdet0 = det - u - v;

  vfloat<K> pdet0 = (tmpdet0 ^ u) | (u ^ v);

  // Return if negative
  vfloat<K> rdet = rcp(det);
  u = u * det;
  v = v * det;
  vfloat<K> t = (dett * rdet) | signmsk(pdet0);

  vbool<K> valid = valid0;
  valid &= (ray.tnear() < t) & (t <= ray.tfar);
  if (likely(none(valid)))
    return false;
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
  Vec3vf<M> o = Vec3vf<M>(ray.org);
  Vec3vf<M> d = Vec3vf<M>(ray.dir);

  vfloat<M> det = dot(tri.n0, d);
  vfloat<M> dett = tri.d0 - dot(o, tri.n0);
  Vec3vf<M> wr = o * det + d * dett;

  vfloat<M> u = dot(wr, tri.n1) + det * tri.d1;
  vfloat<M> v = dot(wr, tri.n2) + det * tri.d2;
  vfloat<M> tmpdet0 = det - u - v;

  vfloat<M> pdet0 = _mm_or_ps((tmpdet0 ^ u), (u ^ v));

  vfloat<M> rdet = rcp(det);
  u = u * rdet;
  v = v * rdet;
  vfloat<M> t = _mm_or_ps(dett * rdet, signmsk(pdet0));

  vfloat<M> tnear = vfloat<M>(ray.tnear());
  vfloat<M> tfar = vfloat<M>(ray.tfar);
  valid &= (tnear < t) & (t <= tfar);
  if (likely(none(valid)))
    return false;

  new (&hit) MTHitM<M>(valid, u, v, t, tri.n0);
  return true;
}

// Crown 9.66
