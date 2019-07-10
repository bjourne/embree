#pragma once

#include "../common/ray.h"
#include "../common/context.h"
#include "filter.h"

namespace embree
{
namespace isa
{

template<int M, int Mx, bool filter>
struct Intersect1EpilogM
{
    RayHit& ray;
    IntersectContext* context;
    const vuint<M>& geomIDs;
    const vuint<M>& primIDs;

    __forceinline Intersect1EpilogM(RayHit& ray,
                                    IntersectContext* context,
                                    const vuint<M>& geomIDs,
                                    const vuint<M>& primIDs)
        : ray(ray), context(context), geomIDs(geomIDs), primIDs(primIDs) {}

    template<typename Hit>
    __forceinline bool operator() (const vbool<Mx>& valid_i, Hit& hit) const
    {
        Scene* scene = context->scene;
        vbool<Mx> valid = valid_i;
        if (Mx > M)
            valid &= (1 << M)-1;
        hit.finalize();
        size_t i = select_min(valid,hit.vt);
        unsigned int geomID = geomIDs[i];

        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        //printf("defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)\n");
        bool foundhit = false;
        goto entry;
        while (true)
        {
            if (unlikely(none(valid)))
                return foundhit;
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
        return true;
    }
};

}
}
