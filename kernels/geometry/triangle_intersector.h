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
        STAT3(normal.trav_prims,1,1,1);

        MoellerTrumboreHitM<M> hit;
        vbool<M> valid = true;
        const Vec3<vfloat<M>> tri_Ng = cross(tri.e2, tri.e1);
        const vuint<M>& geomIDs = tri.geomID();
        const vuint<M>& primIDs = tri.primID();
        if (likely(pre.intersectInternal(valid, ray,
                                         tri.v0, tri.e1, tri.e2,
                                         tri_Ng, hit))) {

            Scene* scene = context->scene;
            vbool<Mx> valid = hit.valid;
            if (Mx > M)
                valid &= (1 << M) - 1;
            hit.finalize();
            size_t i = select_min(valid, hit.vt);
            unsigned int geomID = geomIDs[i];

            /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
            //printf("defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)\n");
            //bool foundhit = false;
            goto entry;
            while (true)
            {
                if (unlikely(none(valid)))
                    return;
                //return foundhit;
                i = select_min(valid, hit.vt);

                geomID = geomIDs[i];
            entry:
                Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
                //printf("defined(EMBREE_RAY_MASK)\n");
                /* goto next hit if mask test fails */
                if ((geometry->mask & ray.mask) == 0) {
                    clear(valid,i);
                    continue;
                }
#endif

                break;
            }
#endif
            /* update hit information */
            const Vec2f uv = hit.uv(i);
            ray.tfar = hit.vt[i];
            ray.Ng.x = hit.vNg.x[i];
            ray.Ng.y = hit.vNg.y[i];
            ray.Ng.z = hit.vNg.z[i];
            ray.u = uv.x;
            ray.v = uv.y;
            ray.primID = primIDs[i];
            ray.geomID = geomID;
            ray.instID = context->instID;
        }
    }

    /*! Test if the ray is occluded by one of M triangles. */
    static __forceinline bool
    occluded(const MoellerTrumboreIntersector1<Mx>& pre,
             Ray& ray,
             IntersectContext* context,
             const TriangleM<M>& tri)
    {
        assert(false);
        return true;
    }
};


}
}
