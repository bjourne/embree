// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "triangle.h"
#include "intersector_epilog.h"

/*! This intersector implements a modified version of the Moeller
 *  Trumbore intersector from the paper "Fast, Minimum Storage
 *  Ray-Triangle Intersection". In contrast to the paper we
 *  precalculate some factors and factor the calculations differently
 *  to allow precalculating the cross product e1 x e2. The resulting
 *  algorithm is similar to the fastest one of the paper "Optimizing
 *  Ray-Triangle Intersection via Automated Search". */

namespace embree
{
  namespace isa
  {
    template<int M>
    struct MoellerTrumboreHitM
    {
      __forceinline MoellerTrumboreHitM() {}

      __forceinline MoellerTrumboreHitM(const vbool<M>& valid,
                                        const vfloat<M>& U,
                                        const vfloat<M>& V,
                                        const vfloat<M>& T,
                                        const vfloat<M>& absDen,
                                        const Vec3vf<M>& Ng)
        : U(U), V(V), T(T), absDen(absDen), valid(valid), vNg(Ng) {}

      __forceinline void finalize()
      {
        const vfloat<M> rcpAbsDen = rcp(absDen);
        vt = T * rcpAbsDen;
        vu = U * rcpAbsDen;
        vv = V * rcpAbsDen;
      }

      __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
      __forceinline float t  (const size_t i) const { return vt[i]; }
      __forceinline Vec3fa Ng(const size_t i) const { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }

    public:
      vfloat<M> U;
      vfloat<M> V;
      vfloat<M> T;
      vfloat<M> absDen;

    public:
      vbool<M> valid;
      vfloat<M> vu;
      vfloat<M> vv;
      vfloat<M> vt;
      Vec3vf<M> vNg;
    };

    template<int M>
    struct MoellerTrumboreIntersector1
    {
      __forceinline MoellerTrumboreIntersector1() {}

      __forceinline MoellerTrumboreIntersector1(const Ray& ray, const void* ptr) {}

      __forceinline bool intersect(const vbool<M>& valid0,
                                   Ray& ray,
                                   const TriangleM<M>& tri,
                                   MoellerTrumboreHitM<M>& hit) const
      {
        /* calculate denominator */
        vbool<M> valid = valid0;
        const Vec3vf<M> O = Vec3vf<M>(ray.org);
        const Vec3vf<M> D = Vec3vf<M>(ray.dir);
        const Vec3vf<M> C = Vec3vf<M>(tri.v0) - O;
        const Vec3vf<M> R = cross(C,D);
        const Vec3vf<M> tri_Ng = cross(tri.e2, tri.e1);
        const vfloat<M> den = dot(Vec3vf<M>(tri_Ng),D);
        const vfloat<M> absDen = abs(den);
        const vfloat<M> sgnDen = signmsk(den);

        /* perform edge tests */
        const vfloat<M> U = dot(R,Vec3vf<M>(tri.e2)) ^ sgnDen;
        const vfloat<M> V = dot(R,Vec3vf<M>(tri.e1)) ^ sgnDen;

        /* perform backface culling */
#if defined(EMBREE_BACKFACE_CULLING)
        valid &= (den < vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
        valid &= (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
        if (likely(none(valid))) return false;

        /* perform depth test */
        const vfloat<M> T = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
        valid &= (absDen*vfloat<M>(ray.tnear()) < T) & (T <= absDen*vfloat<M>(ray.tfar));
        if (likely(none(valid))) return false;

        /* update hit information */
        new (&hit) MoellerTrumboreHitM<M>(valid,U,V,T,absDen,tri_Ng);
        return true;
      }
    };

    template<int K>
    struct MoellerTrumboreHitK
    {
      __forceinline MoellerTrumboreHitK(const vfloat<K>& U,
                                        const vfloat<K>& V,
                                        const vfloat<K>& T,
                                        const vfloat<K>& absDen,
                                        const Vec3vf<K>& Ng)
        : U(U), V(V), T(T), absDen(absDen), Ng(Ng) {}

      __forceinline std::tuple<vfloat<K>,vfloat<K>,vfloat<K>,Vec3vf<K>>
      finalizeK () const
      {
        const vfloat<K> rcpAbsDen = rcp(absDen);
        const vfloat<K> t = T * rcpAbsDen;
        const vfloat<K> u = U * rcpAbsDen;
        const vfloat<K> v = V * rcpAbsDen;
        return std::make_tuple(u,v,t,Ng);
      }

    private:
      const vfloat<K> U;
      const vfloat<K> V;
      const vfloat<K> T;
      const vfloat<K> absDen;
      const Vec3vf<K> Ng;
    };

    template<int M, int K>
    struct MoellerTrumboreIntersectorK
    {
      __forceinline
      MoellerTrumboreIntersectorK(const vbool<K>& valid,
                                  const RayK<K>& ray)
      {
      }

      /*! Intersects K rays with one of M triangles. */
      template<typename Epilog>
      __forceinline void
      intersectK(const vbool<K>& valid0,
                 const RayHitK<K>& ray,
                 const TriangleM<M>& tri,
                 size_t i,
                 const Epilog& epilog) const
      {

        // Do broadcasting and cross prod.
        Vec3<vfloat<K>> p0 = broadcast<vfloat<K>>(tri.v0, i);
        Vec3<vfloat<K>> e1 = broadcast<vfloat<K>>(tri.e1,i);
        Vec3<vfloat<K>> e2 = broadcast<vfloat<K>>(tri.e2,i);
        Vec3<vfloat<K>> tri_Ng = cross(e2, e1);

        /* calculate denominator */
        vbool<K> valid = valid0;
        const Vec3vf<K> C = p0 - ray.org;
        const Vec3vf<K> R = cross(C,ray.dir);
        const vfloat<K> den = dot(tri_Ng,ray.dir);
        const vfloat<K> absDen = abs(den);
        const vfloat<K> sgnDen = signmsk(den);

        /* test against edge p2 p0 */
        const vfloat<K> U = dot(e2, R) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid)))
          return;

        /* test against edge p0 p1 */
        const vfloat<K> V = dot(e1, R) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid)))
          return;

        /* test against edge p1 p2 */
        const vfloat<K> W = absDen-U-V;
        valid &= W >= 0.0f;
        if (likely(none(valid)))
          return;

        /* perform depth test */
        const vfloat<K> T = dot(tri_Ng,C) ^ sgnDen;
        valid &= (absDen*ray.tnear() < T) & (T <= absDen*ray.tfar);
        if (unlikely(none(valid)))
          return;

        /* perform backface culling */
#if defined(EMBREE_BACKFACE_CULLING)
        valid &= den < vfloat<K>(zero);
        if (unlikely(none(valid)))
          return;
#else
        valid &= den != vfloat<K>(zero);
        if (unlikely(none(valid)))
          return;
#endif

        /* calculate hit information */
        MoellerTrumboreHitK<K> hit(U,V,T,absDen,tri_Ng);
        epilog(valid,hit);
      }

      /*! Intersect k'th ray from ray packet of size K with M triangles. */
      __forceinline bool intersectEdge(RayK<K>& ray,
                                       size_t k,
                                       const TriangleM<M>& tri,
                                       MoellerTrumboreHitM<M>& hit) const
      {
        /* calculate denominator */
        typedef Vec3vf<M> Vec3vfM;
        const Vec3vf<M> tri_Ng = cross(tri.e2,tri.e1);

        const Vec3vfM O = broadcast<vfloat<M>>(ray.org,k);
        const Vec3vfM D = broadcast<vfloat<M>>(ray.dir,k);
        const Vec3vfM C = Vec3vfM(tri.v0) - O;
        const Vec3vfM R = cross(C,D);
        const vfloat<M> den = dot(Vec3vfM(tri_Ng),D);
        const vfloat<M> absDen = abs(den);
        const vfloat<M> sgnDen = signmsk(den);

        /* perform edge tests */
        const vfloat<M> U = dot(Vec3vf<M>(tri.e2),R) ^ sgnDen;
        const vfloat<M> V = dot(Vec3vf<M>(tri.e1),R) ^ sgnDen;

        /* perform backface culling */
#if defined(EMBREE_BACKFACE_CULLING)
        vbool<M> valid = (den < vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
        vbool<M> valid = (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
        if (likely(none(valid)))
          return false;

        /* perform depth test */
        const vfloat<M> T = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
        valid &= (absDen*vfloat<M>(ray.tnear()[k]) < T) & (T <= absDen*vfloat<M>(ray.tfar[k]));
        if (likely(none(valid)))
          return false;

        /* calculate hit information */
        new (&hit) MoellerTrumboreHitM<M>(valid,U,V,T,absDen,tri_Ng);
        return true;
      }

      template<typename Epilog>
      __forceinline bool
      intersectEdge(RayK<K>& ray,
                    size_t k,
                    const TriangleM<M>& tri,
                    const Epilog& epilog) const
      {
        MoellerTrumboreHitM<M> hit;
        if (likely(intersectEdge(ray, k, tri, hit)))
          return epilog(hit.valid,hit);
        return false;
      }
    };
  }
}
