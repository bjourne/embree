#pragma once

#include "triangle.h"
#include "triangle_intersector_pluecker.h"
#include "triangle_intersector_moeller.h"

namespace embree
{
namespace isa
{
// /*! Intersects M triangles with 1 ray */
// template<int M, int Mx, bool filter>
// struct TriangleMvIntersector1Moeller
// {
//     typedef TriangleMv<M> Primitive;
//     typedef MoellerTrumboreIntersector1<Mx> Precalculations;

//     /*! Intersect a ray with M triangles and updates the hit. */
//     static __forceinline void intersect(Precalculations& pre,
//                                         RayHit& ray,
//                                         IntersectContext* context,
//                                         const Primitive& tri)
//     {
//         STAT3(normal.trav_prims,1,1,1);
//         pre.intersect(
//             ray,tri.v0,tri.v1,tri.v2,/*UVIdentity<Mx>(),*/
//             Intersect1EpilogM<M,Mx,filter>(
//                 ray,context,tri.geomID(),tri.primID()));
//     }

//     /*! Test if the ray is occluded by one of the M triangles. */
//     static __forceinline bool
//     occluded(const Precalculations& pre,
//              Ray& ray,
//              IntersectContext* context,
//              const Primitive& tri)
//     {
//         STAT3(shadow.trav_prims,1,1,1);
//         return pre.intersect(ray,tri.v0,tri.v1,tri.v2,/*UVIdentity<Mx>(),*/Occluded1EpilogM<M,Mx,filter>(ray,context,tri.geomID(),tri.primID()));
//     }
// };


/*! Intersects M triangles with 1 ray */
template<int M, int Mx, bool filter>
struct TriangleMvIntersector1Pluecker
{
    typedef TriangleMv<M> Primitive;
    typedef PlueckerIntersector1<Mx> Precalculations;

    /*! Intersect a ray with M triangles and updates the hit. */
    static __forceinline void intersect(Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& tri)
    {
        STAT3(normal.trav_prims,1,1,1);
        pre.intersect(ray,tri.v0,tri.v1,tri.v2,UVIdentity<Mx>(),Intersect1EpilogM<M,Mx,filter>(ray,context,tri.geomID(),tri.primID()));
    }

      /*! Test if the ray is occluded by one of the M triangles. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& tri)
      {
        STAT3(shadow.trav_prims,1,1,1);
        return pre.intersect(ray,tri.v0,tri.v1,tri.v2,UVIdentity<Mx>(),Occluded1EpilogM<M,Mx,filter>(ray,context,tri.geomID(),tri.primID()));
      }
    };

}
}
