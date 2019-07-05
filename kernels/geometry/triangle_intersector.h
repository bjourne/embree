#pragma once

#include "triangle.h"
#include "triangle_intersector_moeller.h"

namespace embree
{
namespace isa
{
/*! Intersects M triangles with 1 ray */
template<int M, int Mx, bool filter>
struct TriangleMIntersector1Moeller
{
    typedef TriangleM<M> Primitive;
    typedef MoellerTrumboreIntersector1<Mx> Precalculations;

    /*! Intersect a ray with the M triangles and updates the hit. */
    static __forceinline void
    intersect(const MoellerTrumboreIntersector1<Mx>& pre,
              RayHit& ray,
              IntersectContext* context,
              const TriangleM<M>& tri)
    {
        //printf("TriangleMIntersector1Moeller::intersect\n");
        STAT3(normal.trav_prims,1,1,1);
        pre.intersectEdge(ray, tri.v0, tri.e1, tri.e2,
                          Intersect1EpilogM<M,Mx,filter>(ray,
                                                         context,
                                                         tri.geomID(),
                                                         tri.primID()));
    }

    /*! Test if the ray is occluded by one of M triangles. */
    static __forceinline bool
    occluded(const MoellerTrumboreIntersector1<Mx>& pre,
             Ray& ray,
             IntersectContext* context,
             const TriangleM<M>& tri)
    {
        STAT3(shadow.trav_prims,1,1,1);
        return pre.intersectEdge(
            ray,tri.v0,tri.e1,tri.e2,
            Occluded1EpilogM<M,Mx,filter>(
                ray,context,tri.geomID(),tri.primID()));
    }
};

#if defined(__AVX__)
template<bool filter>
struct TriangleMIntersector1Moeller<4,8,filter>
{
    static const size_t M = 4;
    static const size_t Mx = 8;

    typedef TriangleM<M> Primitive;
    typedef MoellerTrumboreIntersector1<Mx> Precalculations;

    static __forceinline void
    intersect(const Precalculations& pre,
              RayHit& ray,
              IntersectContext* context,
              const TriangleM<M>& tri)
    {
        printf("__AVX__ defined\n");
        STAT3(normal.trav_prims,1,1,1);
        pre.intersect(
            ray,tri.v0,tri.e1,tri.e2,
            Intersect1EpilogM<M,Mx,filter>(
                ray,context,tri.geomID(),tri.primID()));
    }

    /*! Test if the ray is occluded by one of M triangles. */
    static __forceinline bool
    occluded(const Precalculations& pre,
             Ray& ray,
             IntersectContext* context,
             const TriangleM<M>& tri)
    {
        STAT3(shadow.trav_prims,1,1,1);
        return pre.intersect(
            ray,tri.v0,tri.e1,tri.e2,
            Occluded1EpilogM<M,Mx,filter>(
                ray,context,tri.geomID(),tri.primID()));
    }
};
#endif

}
}
