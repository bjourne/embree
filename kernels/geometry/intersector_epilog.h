#pragma once

#include "../common/ray.h"
#include "../common/context.h"
#include "filter.h"

namespace embree
{
  namespace isa
  {
    template<int M, int Mx, int K, bool filter>
    struct Intersect1KEpilogM
    {
      RayHitK<K>& ray;
      size_t k;
      IntersectContext* context;
      const TriangleM<M>& tri;

      __forceinline Intersect1KEpilogM(RayHitK<K>& ray, size_t k,
                                       IntersectContext* context,
                                       const TriangleM<M>& tri)
        : ray(ray), k(k), context(context), tri(tri) {}

      template<typename Hit>
      __forceinline bool operator() (const vbool<Mx>& valid_i, Hit& hit) const
      {
        Scene* scene = context->scene;
        vbool<Mx> valid = valid_i;
        hit.finalize();
        if (Mx > M) valid &= (1<<M)-1;
        size_t i = select_min(valid,hit.vt);
        assert(i<M);
        unsigned int geomID = tri.geomIDs[i];

        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        bool foundhit = false;
        goto entry;
        while (true)
        {
          if (unlikely(none(valid))) return foundhit;
          i = select_min(valid,hit.vt);
          assert(i<M);
          geomID = tri.geomIDs[i];
        entry:
          Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
          /* goto next hit if mask test fails */
          if ((geometry->mask & ray.mask[k]) == 0) {
            clear(valid,i);
            continue;
          }
#endif

#if defined(EMBREE_FILTER_FUNCTION)
          /* call intersection filter function */
          if (filter) {
            if (unlikely(context->hasContextFilter() ||
                         geometry->hasIntersectionFilter())) {
              assert(i<M);
              const Vec2f uv = Vec2f(hit.vu[i], hit.vv[i]);
              HitK<K> h(context->instID,
                        geomID, tri.primIDs[i],
                        uv.x,uv.y,
                        hit.Ng(i));
              const float old_t = ray.tfar[k];
              ray.tfar[k] = hit.vt[i];
              const bool found = any(runIntersectionFilter(vbool<K>(1<<k),geometry,ray,context,h));
              if (!found) ray.tfar[k] = old_t;
              foundhit = foundhit | found;
              clear(valid,i);
              valid &= hit.vt <= ray.tfar[k]; // intersection filters may modify tfar value
              continue;
            }
          }
#endif
          break;
        }
#endif
        assert(i<M);
        /* update hit information */
#if 0 && defined(__AVX512F__) // do not enable, this reduced frequency for BVH4
        ray.updateK(i,k,hit.vt,hit.vu,hit.vv,vfloat<Mx>(hit.vNg.x),vfloat<Mx>(hit.vNg.y),vfloat<Mx>(hit.vNg.z),geomID,vuint<Mx>(primIDs));
        ray.instID[k] = context->instID;
#else
        const Vec2f uv = Vec2f(hit.vu[i], hit.vv[i]);
        ray.tfar[k] = hit.vt[i];
        ray.Ng.x[k] = hit.vNg.x[i];
        ray.Ng.y[k] = hit.vNg.y[i];
        ray.Ng.z[k] = hit.vNg.z[i];
        ray.u[k] = uv.x;
        ray.v[k] = uv.y;
        ray.primID[k] = tri.primIDs[i];
        ray.geomID[k] = geomID;
        ray.instID[k] = context->instID;
#endif
        return true;
      }
    };

    template<int M, int Mx, int K, bool filter>
    struct Occluded1KEpilogM
    {
      RayK<K>& ray;
      size_t k;
      IntersectContext* context;
      const TriangleM<M>& tri;

      __forceinline Occluded1KEpilogM(RayK<K>& ray, size_t k,
                                      IntersectContext* context,
                                      const TriangleM<M>& tri)
        : ray(ray), k(k), context(context), tri(tri) {}

      template<typename Hit>
      __forceinline bool
      operator() (const vbool<Mx>& valid_i, Hit& hit) const
      {
        Scene* scene = context->scene;

        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        if (unlikely(filter))
          hit.finalize(); /* called only once */

        vbool<Mx> valid = valid_i;
        if (Mx > M) valid &= (1<<M)-1;
        size_t m = movemask(valid);
        goto entry;
        while (true)
        {
          if (unlikely(m == 0)) return false;
        entry:
          size_t i=bsf(m);

          const unsigned int geomID = tri.geomIDs[i];
          Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
          /* goto next hit if mask test fails */
          if ((geometry->mask & ray.mask[k]) == 0) {
            m=btc(m,i);
            continue;
          }
#endif

#if defined(EMBREE_FILTER_FUNCTION)
          /* execute occlusion filer */
          if (filter) {
            if (unlikely(context->hasContextFilter() || geometry->hasOcclusionFilter()))
            {
              const Vec2f uv = Vec2f(hit.vu[i], hit.vv[i]);
              const float old_t = ray.tfar[k];
              ray.tfar[k] = hit.vt[i];
              HitK<K> h(context->instID,
                        geomID, tri.primIDs[i],
                        uv.x, uv.y,
                        hit.Ng(i));
              if (any(runOcclusionFilter(vbool<K>(1<<k),geometry,ray,context,h))) return true;
              ray.tfar[k] = old_t;
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
