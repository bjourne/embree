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
#include "triangle_intersector_moeller.h"

namespace embree
{
  namespace isa
  {
    template<int M, int Mx>
    static __forceinline bool
    occludedEpilog(Ray& ray,
                   IntersectContext* context,
                   const vbool<Mx>& valid_i,
                   const TriangleM<M>& tri,
                   MoellerTrumboreHitM<M>& hit)
    {
      Scene* scene = context->scene;
      /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
      hit.finalize(); /* called only once */

      vbool<Mx> valid = valid_i;
      if (Mx > M)
        valid &= (1<<M)-1;
      size_t m=movemask(valid);
      goto entry;
      while (true) {
        if (unlikely(m == 0))
          return false;
      entry:
        size_t i=bsf(m);

        const unsigned int geomID = tri.geomIDs[i];
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
        if (unlikely(context->hasContextFilter() ||
                     geometry->hasOcclusionFilter())) {
          const Vec2f uv = Vec2f(hit.vu[i], hit.vv[i]);
          HitK<1> h(context->instID,
                    geomID,
                    tri.primIDs[i],
                    uv.x, uv.y,
                    hit.Ng(i));
          const float old_t = ray.tfar;
          ray.tfar = hit.vt[i];
          if (runOcclusionFilter1(geometry,ray,context,h))
            return true;
          ray.tfar = old_t;
          m=btc(m,i);
          continue;
        }
#endif
        break;
      }
#endif
      return true;
    }

    template<int M, int Mx>
    static __forceinline bool
    intersectEpilog(RayHit& ray,
                    IntersectContext* context,
                    const vbool<Mx>& valid_i,
                    const TriangleM<M>& tri,
                    MoellerTrumboreHitM<M>& hit)
    {
      Scene* scene = context->scene;
      vbool<Mx> valid = valid_i;
      if (Mx > M)
        valid &= (1<<M)-1;
      hit.finalize();
      size_t i = select_min(valid,hit.vt);
      unsigned int geomID = tri.geomIDs[i];

      /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
      bool foundhit = false;
      goto entry;
      while (true)
      {
        if (unlikely(none(valid))) return foundhit;
        i = select_min(valid,hit.vt);

        geomID = tri.geomIDs[i];
      entry:
        Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
        /* goto next hit if mask test fails */
        if ((geometry->mask & ray.mask) == 0) {
          clear(valid,i);
          continue;
        }
#endif

#if defined(EMBREE_FILTER_FUNCTION)
        /* call intersection filter function */
        if (unlikely(context->hasContextFilter() || geometry->hasIntersectionFilter())) {
          const Vec2f uv = Vec2f(hit.vu[i], hit.vv[i]);
          HitK<1> h(context->instID,
                    geomID, tri.primIDs[i],
                    uv.x,uv.y,
                    hit.Ng(i));
          const float old_t = ray.tfar;
          ray.tfar = hit.vt[i];
          const bool found = runIntersectionFilter1(geometry,ray,context,h);
          if (!found) ray.tfar = old_t;
          foundhit |= found;
          clear(valid,i);
          valid &= hit.vt <= ray.tfar; // intersection filters may modify tfar value
          continue;
        }
#endif
        break;
      }
#endif

      /* update hit information */
      const Vec2f uv = Vec2f(hit.vu[i], hit.vv[i]);
      ray.tfar = hit.vt[i];
      ray.Ng.x = hit.vNg.x[i];
      ray.Ng.y = hit.vNg.y[i];
      ray.Ng.z = hit.vNg.z[i];
      ray.u = uv.x;
      ray.v = uv.y;
      ray.primID = tri.primIDs[i];
      ray.geomID = geomID;
      ray.instID = context->instID;
      return true;
    }

    /*! Intersects M triangles with 1 ray */
    template<int M, int Mx>
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
        STAT3(normal.trav_prims, 1, 1, 1);
        MoellerTrumboreHitM<M> hit;
        vbool<M> valid = true;
        if (likely(pre.intersect(valid, ray, tri, hit))) {

          intersectEpilog<M, Mx>(ray, context, hit.valid,
                                 tri, hit);
        }
      }

      /*! Test if the ray is occluded by one of M triangles. */
      static __forceinline bool
      occluded(const MoellerTrumboreIntersector1<Mx>& pre,
               Ray& ray,
               IntersectContext* context,
               const TriangleM<M>& tri)
      {
        STAT3(shadow.trav_prims, 1, 1, 1);
        MoellerTrumboreHitM<M> hit;
        vbool<M> valid = true;
        if (likely(pre.intersect(valid, ray, tri, hit))) {
          return occludedEpilog<M, Mx>(ray, context,
                                       hit.valid, tri, hit);
        }
        return false;
      }
    };

    /*! Intersects M triangles with K rays. */
    template<int M, int Mx, int K, bool filter>
    struct TriangleMIntersectorKMoeller
    {
      typedef TriangleM<M> Primitive;
      typedef MoellerTrumboreIntersectorK<Mx,K> Precalculations;



      /*! Intersects K rays with M triangles. */
      static __forceinline
      void intersect(const vbool<K>& valid_i,
                     Precalculations& pre,
                     RayHitK<K>& ray,
                     IntersectContext* context,
                     const TriangleM<M>& tri)
      {
        //printf("TriangleMIntersectorKMoeller::intersect LOOP\n");
        STAT_USER(0,TriangleM<M>::max_size());
        for (size_t i = 0; i < TriangleM<M>::max_size(); i++)
        {
          if (!tri.valid(i))
            break;
          STAT3(normal.trav_prims, 1, popcnt(valid_i), K);
          pre.intersectK(
            valid_i,
            ray,
            tri, i,
            IntersectKEpilogM<M,K,filter>(ray,context,tri.geomID(),tri.primID(),i));
        }
      }

      /*! Intersect a ray with M triangles and updates the hit. */
      static __forceinline void
      intersect(Precalculations& pre,
                RayHitK<K>& ray,
                size_t k,
                IntersectContext* context,
                const TriangleM<M>& tri)
      {
        STAT3(normal.trav_prims,1,1,1);
        pre.intersectEdge(
          ray, k,
          tri,
          Intersect1KEpilogM<M,Mx,K,filter>(ray,k,context,tri.geomID(),tri.primID()));
      }

      /*! Test for K rays if they are occluded by any of the M triangles. */
      static __forceinline vbool<K>
      occluded(const vbool<K>& valid_i,
               Precalculations& pre,
               RayK<K>& ray,
               IntersectContext* context,
               const TriangleM<M>& tri)
      {
        vbool<K> valid0 = valid_i;

        for (size_t i=0; i<TriangleM<M>::max_size(); i++)
        {
          if (!tri.valid(i))
            break;
          STAT3(shadow.trav_prims,1,popcnt(valid0),K);
          pre.intersectK(
            valid0,
            ray,
            tri, i,
            OccludedKEpilogM<M,K,filter>(valid0,ray,context,tri.geomID(),tri.primID(),i));

          if (none(valid0))
            break;
        }
        return !valid0;
      }

      /*! Test if the ray is occluded by one of the M triangles. */
      static __forceinline bool occluded(Precalculations& pre,
                                         RayK<K>& ray,
                                         size_t k,
                                         IntersectContext* context,
                                         const TriangleM<M>& tri)
      {
        STAT3(shadow.trav_prims,1,1,1);
        return pre.intersectEdge(
          ray, k,
          tri,
          Occluded1KEpilogM<M,Mx,K,filter>(ray,k,context,tri.geomID(),tri.primID()));
      }
    };
  }
}
