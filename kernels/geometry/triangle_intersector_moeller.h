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
    __forceinline
    MoellerTrumboreHitM() {}

    __forceinline
    MoellerTrumboreHitM(const vbool<M>& valid,
                        const vfloat<M>& U,
                        const vfloat<M>& V,
                        const vfloat<M>& T,
                        const vfloat<M>& absDen,
                        const Vec3vf<M>& Ng)
        : U(U), V(V), T(T), absDen(absDen), valid(valid), vNg(Ng) {}

    __forceinline void
    finalize()
    {
        const vfloat<M> rcpAbsDen = rcp(absDen);
        vt = T * rcpAbsDen;
        vu = U * rcpAbsDen;
        vv = V * rcpAbsDen;
    }

    __forceinline Vec2f
    uv(const size_t i) const
    {
        return Vec2f(vu[i],vv[i]);
    }
    __forceinline float
    t(const size_t i) const
    {
        return vt[i];
    }
    __forceinline Vec3fa
    Ng(const size_t i) const
    {
        return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]);
    }

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
    __forceinline MoellerTrumboreIntersector1()
    {
    }

    __forceinline MoellerTrumboreIntersector1(const Ray& ray,
                                              const void* ptr)
    {
    }

    // __forceinline bool
    // intersect(const vbool<M>& valid0,
    //           Ray& ray,
    //           const Vec3vf<M>& tri_v0,
    //           const Vec3vf<M>& tri_e1,
    //           const Vec3vf<M>& tri_e2,
    //           const Vec3vf<M>& tri_Ng,
    //           MoellerTrumboreHitM<M>& hit) const
    // {
    //     const Vec3vf<M> v0o = Vec3vf<M>(tri_v0) - O;
    //     const vfloat<M> a = dot(Vec3vf<M>(tri_Ng), v0o);
    //     const vfloat<M> b = dot(Vec3vf<M>(tri_Ng), D);
    //     const vfloat<M> T = a / b;
    // }

    // what is valid0?
    __forceinline bool
    intersect(const vbool<M>& valid0,
              Ray& ray,
              const Vec3vf<M>& tri_v0,
              const Vec3vf<M>& tri_e1,
              const Vec3vf<M>& tri_e2,
              const Vec3vf<M>& tri_Ng,
              MoellerTrumboreHitM<M>& hit) const
    {
        /* calculate denominator */
        vbool<M> valid = valid0;
        const Vec3vf<M> O = Vec3vf<M>(ray.org);
        const Vec3vf<M> D = Vec3vf<M>(ray.dir);
        const Vec3vf<M> C = Vec3vf<M>(tri_v0) - O;

        const Vec3<vfloat<M>> R = cross(C, D);

        const vfloat<M> den = dot(Vec3vf<M>(tri_Ng),D);
        const vfloat<M> absDen = abs(den);
        const vfloat<M> sgnDen = signmsk(den);

        /* perform edge tests */
        const vfloat<M> U = dot(R, Vec3vf<M>(tri_e2)) ^ sgnDen;
        const vfloat<M> V = dot(R, Vec3vf<M>(tri_e1)) ^ sgnDen;

        // No backface culling
        valid &= (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
        if (likely(none(valid)))
            return false;

        /* perform depth test */
        const vfloat<M> T = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
        valid &= (absDen*vfloat<M>(ray.tnear()) < T) & (T <= absDen*vfloat<M>(ray.tfar));
        if (likely(none(valid)))
            return false;

        /* update hit information */
        new (&hit) MoellerTrumboreHitM<M>(valid, U, V, T, absDen, tri_Ng);
        return true;
    }

    __forceinline bool
    intersectEdge(Ray& ray,
                  const Vec3vf<M>& tri_v0,
                  const Vec3vf<M>& tri_e1,
                  const Vec3vf<M>& tri_e2,
                  MoellerTrumboreHitM<M>& hit) const
    {
          vbool<M> valid = true;
          const Vec3<vfloat<M>> tri_Ng = cross(tri_e2,tri_e1);
          return intersect(valid, ray, tri_v0, tri_e1, tri_e2, tri_Ng, hit);
    }

    __forceinline bool
    intersect(Ray& ray,
              const Vec3vf<M>& v0,
              const Vec3vf<M>& v1,
              const Vec3vf<M>& v2,
              MoellerTrumboreHitM<M>& hit) const
    {
        const Vec3vf<M> e1 = v0-v1;
        const Vec3vf<M> e2 = v2-v0;
        return intersectEdge(ray,v0,e1,e2,hit);
    }

    __forceinline bool
    intersect(const vbool<M>& valid,
              Ray& ray,
              const Vec3vf<M>& v0,
              const Vec3vf<M>& v1,
              const Vec3vf<M>& v2,
              MoellerTrumboreHitM<M>& hit) const
    {
        const Vec3vf<M> e1 = v0-v1;
        const Vec3vf<M> e2 = v2-v0;
        return intersectEdge(valid,ray,v0,e1,e2,hit);
    }

    template<typename Epilog>
    __forceinline bool
    intersectEdge(Ray& ray,
                  const Vec3vf<M>& v0,
                  const Vec3vf<M>& e1,
                  const Vec3vf<M>& e2,
                  const Epilog& epilog) const
    {
        MoellerTrumboreHitM<M> hit;
        if (likely(intersectEdge(ray,v0,e1,e2,hit))) {
            return epilog(hit.valid, hit);
        }
        return false;
    }

    template<typename Epilog>
    __forceinline bool
    intersect(Ray& ray,
              const Vec3vf<M>& v0,
              const Vec3vf<M>& v1,
              const Vec3vf<M>& v2,
              const Epilog& epilog) const
    {
        MoellerTrumboreHitM<M> hit;
        if (likely(intersect(ray,v0,v1,v2,hit)))
            return epilog(hit.valid,hit);
        return false;
    }

    // template<typename Epilog>
    // __forceinline bool
    // intersect(const vbool<M>& valid,
    //           Ray& ray,
    //           const Vec3vf<M>& v0,
    //           const Vec3vf<M>& v1,
    //           const Vec3vf<M>& v2,
    //           const Epilog& epilog) const
    // {
    //     MoellerTrumboreHitM<M> hit;
    //     if (likely(intersect(valid, ray, v0, v1, v2, hit)))
    //         return epilog(hit.valid,hit);
    //     return false;
    // }
};


}
}
