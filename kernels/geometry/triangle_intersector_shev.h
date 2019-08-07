#pragma once

template<int K>
static __forceinline bool
isectAlgo(const Vec3vf<K>& o,
          const Vec3vf<K>& d,
          const vfloat<K>& tn, const vfloat<K>& tf,
          const Vec3vf<K>& n0, const Vec3vf<K>& n1, const Vec3vf<K>& n2,
          const vfloat<K>& d0, const vfloat<K>& d1, const vfloat<K>& d2,
          MTHit<K>& hit, const vbool<K>& valid0) {

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
  return false;
}

/*! Intersect 1 ray with one of M triangles. */
template<int M>
static __forceinline bool
intersect1RayMTris(Vec3vf<M> o, Vec3vf<M> d,
                   vfloat<M> tn, vfloat<M> tf,
                   const TriangleM<M>& tri, MTHit<M>& hit)
{
  assert(false);
  return false;
}
