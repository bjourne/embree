#pragma once

#include "triangle.h"
#include "triangle_intersector_moeller.h"

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
/*! Intersects M triangles with 1 ray */
template<int M, int Mx, bool filter>
struct TriangleMIntersector1Moeller
{
    typedef TriangleM<M> Primitive;
    typedef MoellerTrumboreIntersector1<Mx> Precalculations;

    /*! Intersect a ray with the M triangles and updates the hit. */
    static __forceinline void
    intersect(RayHit& ray,
              IntersectContext* context,
              const TriangleM<M>& tri)
    {
        STAT3(normal.trav_prims,1,1,1);

        vbool<M> valid = true;
        Vec3<vfloat<M>> tri_Ng = cross(tri.e2, tri.e1);
        //vuint<M>& geomIDs = tri.geomID();
        //vuint<M>& primIDs = tri.primIDs;

        Vec3<vfloat<M>> O = Vec3<vfloat<M>>(ray.org);
        Vec3<vfloat<M>> D = Vec3<vfloat<M>>(ray.dir);
        Vec3<vfloat<M>> C = Vec3<vfloat<M>>(tri.v0) - O;
        Vec3<vfloat<M>> R = cross(C, D);

        vfloat<M> den = dot(tri_Ng, D);
        vfloat<M> absDen = abs(den);
        vfloat<M> sgnDen = signmsk(den);

        /* perform edge tests */
        vfloat<M> U = dot(R, tri.e2) ^ sgnDen;
        vfloat<M> V = dot(R, tri.e1) ^ sgnDen;

        // No backface culling
        valid &= (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
        if (likely(none(valid)))
            return;

        /* perform depth test */
        vfloat<M> T = dot(Vec3vf<M>(tri_Ng), C) ^ sgnDen;
        valid &= (absDen*vfloat<M>(ray.tnear()) < T) & (T <= absDen*vfloat<M>(ray.tfar));
        if (likely(none(valid)))
            return;

        Scene* scene = context->scene;
        vbool<Mx> valid2 = valid;
        if (Mx > M)
            valid2 &= (1 << M) - 1;

        // Trick to avoid dividing to early.
        vfloat<M> rcpAbsDen = rcp(absDen);
        vfloat<M> vt = T * rcpAbsDen;
        vfloat<M> vu = U * rcpAbsDen;
        vfloat<M> vv = V * rcpAbsDen;

        size_t i = select_min(valid2, vt);
        unsigned int geomID = tri.geomIDs[i];

        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        goto entry;
        while (true)
        {
            if (unlikely(none(valid2)))
                return;
            i = select_min(valid2, vt);

            geomID = tri.geomIDs[i];
        entry:
            Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
                clear(valid2,i);
                continue;
            }
#endif
            break;
        }
#endif
        /* update hit information */
        ray.tfar = vt[i];
        ray.Ng.x = tri_Ng.x[i];
        ray.Ng.y = tri_Ng.y[i];
        ray.Ng.z = tri_Ng.z[i];

        ray.u = vu[i];
        ray.v = vv[i];

        ray.primID = tri.primIDs[i];
        ray.geomID = geomID;
        ray.instID = context->instID;
    }

    /*! Test if the ray is occluded by one of M triangles. */
    static __forceinline bool
    occluded(Ray& ray,
             IntersectContext* context,
             const TriangleM<M>& tri)
    {
        assert(false);
        return true;
    }
};


}
}
