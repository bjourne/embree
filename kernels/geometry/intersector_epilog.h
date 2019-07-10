#pragma once

#include "../common/ray.h"
#include "../common/context.h"
#include "filter.h"

namespace embree
{
namespace isa
{

template<int M>
struct UVIdentity
{
    __forceinline void operator() (vfloat<M>& u, vfloat<M>& v) const {}
};

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
            if (unlikely(none(valid))) return foundhit;
            i = select_min(valid,hit.vt);

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

#if defined(EMBREE_FILTER_FUNCTION)
            //printf("defined(EMBREE_FILTER_FUNCTION)\n");
            /* call intersection filter function */
            if (filter) {
                if (unlikely(context->hasContextFilter() ||
                             geometry->hasIntersectionFilter()))
                {
                    const Vec2f uv = hit.uv(i);
                    HitK<1> h(context->instID,geomID,primIDs[i],uv.x,uv.y,hit.Ng(i));
                    const float old_t = ray.tfar;
                    ray.tfar = hit.t(i);
                    const bool found = runIntersectionFilter1(geometry,ray,context,h);
                    if (!found) ray.tfar = old_t;
                    foundhit |= found;
                    clear(valid,i);
                    // intersection filters may modify tfar value
                    valid &= hit.vt <= ray.tfar;
                    continue;
                }
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

template<int M, int Mx, bool filter>
struct Occluded1EpilogM
{
    Ray& ray;
    IntersectContext* context;
    const vuint<M>& geomIDs;
    const vuint<M>& primIDs;

    __forceinline Occluded1EpilogM(Ray& ray,
                                   IntersectContext* context,
                                   const vuint<M>& geomIDs,
                                   const vuint<M>& primIDs)
        : ray(ray), context(context), geomIDs(geomIDs), primIDs(primIDs) {}

    template<typename Hit>
    __forceinline bool operator() (const vbool<Mx>& valid_i, Hit& hit) const
    {
        Scene* scene = context->scene;
        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        if (unlikely(filter))
            hit.finalize(); /* called only once */

        vbool<Mx> valid = valid_i;
        if (Mx > M) valid &= (1<<M)-1;
        size_t m=movemask(valid);
        goto entry;
        while (true)
        {
            if (unlikely(m == 0)) return false;
        entry:
            size_t i=bsf(m);

            const unsigned int geomID = geomIDs[i];
            Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
                m=btc(m,i);
                continue;
            }
#endif

#if defined(EMBREE_FILTER_FUNCTION)
            /* if we have no filter then the test passed */
            if (filter) {
                if (unlikely(context->hasContextFilter() || geometry->hasOcclusionFilter()))
                {
                    const Vec2f uv = hit.uv(i);
                    HitK<1> h(context->instID,geomID,primIDs[i],uv.x,uv.y,hit.Ng(i));
                    const float old_t = ray.tfar;
                    ray.tfar = hit.t(i);
                    if (runOcclusionFilter1(geometry,ray,context,h)) return true;
                    ray.tfar = old_t;
                    m=btc(m,i);
                    continue;
                }
            }
#endif
            break;
        }
#endif

        return true;
    }
};

}
}
