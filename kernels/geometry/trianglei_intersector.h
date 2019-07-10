#pragma once

#include "trianglei.h"
#include "triangle_intersector_moeller.h"
#include "triangle_intersector_pluecker.h"

namespace embree
{
namespace isa
{
/*! Intersects M triangles with 1 ray */
template<int M, int Mx, bool filter>
struct TriangleMiIntersector1Pluecker
{
    typedef TriangleMi<M> Primitive;
    typedef PlueckerIntersector1<Mx> Precalculations;

    static __forceinline void intersect(const Precalculations& pre,
                                        RayHit& ray,
                                        IntersectContext* context,
                                        const Primitive& tri)
    {
        STAT3(normal.trav_prims,1,1,1);
        Vec3vf<M> v0, v1, v2; tri.gather(v0,v1,v2,context->scene);
        pre.intersect(
            ray,v0,v1,v2,UVIdentity<Mx>(),
            Intersect1EpilogM<M,Mx,filter>(
                ray,context,tri.geomID(),tri.primID()));
    }

    static __forceinline bool occluded(const Precalculations& pre,
                                       Ray& ray,
                                       IntersectContext* context,
                                       const Primitive& tri)
    {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf<M> v0, v1, v2; tri.gather(v0,v1,v2,context->scene);
        return pre.intersect(
            ray,v0,v1,v2,UVIdentity<Mx>(),
            Occluded1EpilogM<M,Mx,filter>(
                ray,context,tri.geomID(),tri.primID()));
    }
};

}
}
