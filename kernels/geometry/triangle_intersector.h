#pragma once

#include "triangle.h"
#include "filter.h"

namespace embree
{
  namespace isa
  {
    /////////////////////////////////////////////////////////////////
    /// Hit types
    /////////////////////////////////////////////////////////////////
    template<int M>
    struct MTHitM
    {
      __forceinline MTHitM() {}
      __forceinline MTHitM(const vbool<M>& valid,
                           const vfloat<M>& U,
                           const vfloat<M>& V,
                           const vfloat<M>& T,
                           const Vec3vf<M>& N)
        : valid(valid), U(U), V(V), T(T), N(N) {}

      __forceinline Vec3fa Ng(const size_t i) const
      {
        return Vec3fa(N.x[i], N.y[i], N.z[i]);
      }

    public:
      vbool<M> valid;
      vfloat<M> U;
      vfloat<M> V;
      vfloat<M> T;
      Vec3vf<M> N;
    };

    template<int K>
    struct MTHitK
    {
      __forceinline MTHitK() {}
      __forceinline MTHitK(const vbool<K>& valid,
                           const vfloat<K>& U,
                           const vfloat<K>& V,
                           const vfloat<K>& T,
                           const Vec3vf<K>& N)
        : valid(valid), U(U), V(V), T(T), N(N) {}
    public:
      const vbool<K> valid;
      const vfloat<K> U;
      const vfloat<K> V;
      const vfloat<K> T;
      const Vec3vf<K> N;
    };

    /////////////////////////////////////////////////////////////////
    /// Intersector functions
    /////////////////////////////////////////////////////////////////
    #if ISECT_METHOD == ISECT_HH
    #include "triangle_intersector_hh.h"
    #elif ISECT_METHOD == ISECT_EMBREE
    #include "triangle_intersector_embree.h"
    #endif

    /////////////////////////////////////////////////////////////////
    /// Epilog functions
    /////////////////////////////////////////////////////////////////
    template<int M, int K, bool filter>
    static __forceinline void
    epilogKRaysMTrisIntersect(IntersectContext* context,
                              RayHitK<K>& ray,
                              size_t i,
                              const vbool<K> valid0,
                              MTHitK<K>& hit,
                              const TriangleM<M>& tri)
    {
      Scene* scene = context->scene;
      vbool<K> valid2 = valid0;

      const unsigned int geomID = tri.geomIDs[i];
      const unsigned int primID = tri.primIDs[i];
      Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

      /* occlusion filter test */
#if defined(EMBREE_FILTER_FUNCTION)
      if (filter) {
        if (unlikely(context->hasContextFilter() ||
                     geometry->hasIntersectionFilter())) {
          HitK<K> h(context->instID, geomID, primID,
                    hit.U, hit.V, hit.N);
          const vfloat<K> old_t = ray.tfar;
          ray.tfar = select(valid2, hit.T, ray.tfar);
          const vbool<K> m_accept = runIntersectionFilter(valid2,
                                                          geometry,
                                                          ray,
                                                          context,
                                                          h);
          ray.tfar = select(m_accept, ray.tfar, old_t);
          return;
        }
      }
#endif

      /* update hit information */
      vfloat<K>::store(valid2, &ray.tfar, hit.T);
      vfloat<K>::store(valid2, &ray.Ng.x, hit.N.x);
      vfloat<K>::store(valid2, &ray.Ng.y, hit.N.y);
      vfloat<K>::store(valid2, &ray.Ng.z, hit.N.z);
      vfloat<K>::store(valid2, &ray.u, hit.U);
      vfloat<K>::store(valid2, &ray.v, hit.V);
      vuint<K>::store(valid2, &ray.primID, primID);
      vuint<K>::store(valid2, &ray.geomID, geomID);
      vuint<K>::store(valid2, &ray.instID, context->instID);
    }


    template<int M, int K, bool filter>
    static __forceinline void
    epilogKRaysMTrisOccluded(IntersectContext* context,
                             RayK<K>& ray,
                             const vbool<K> valid0,
                             MTHitK<K>& hit,
                             const TriangleM<M>& tri,
                             size_t i)
    {
      vbool<K> valid2 = valid0;

      /* ray masking test */
      Scene* scene = context->scene;
      const unsigned int geomID = tri.geomIDs[i];
      const unsigned int primID = tri.primIDs[i];
      Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

      /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION)
      if (filter) {
        if (unlikely(context->hasContextFilter() ||
                     geometry->hasOcclusionFilter())) {
          HitK<K> h(context->instID,
                    geomID, primID,
                    hit.U, hit.V, hit.N);
          const vfloat<K> old_t = ray.tfar;
          ray.tfar = select(valid2, hit.T, ray.tfar);
          valid2 = runOcclusionFilter(valid2, geometry, ray, context, h);
          ray.tfar = select(valid2, ray.tfar, old_t);
        }
      }
#endif

    }

    template<int M, int Mx, int K, bool filter>
    static __forceinline bool
    epilogKthRayMTrisOccluded(IntersectContext* context,
                              RayK<K>& ray,
                              size_t k,
                              const vbool<Mx> valid_i,
                              MTHitM<M>& hit,
                              const TriangleM<M>& tri)
    {
      Scene* scene = context->scene;

      /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION)

      vbool<Mx> valid2 = valid_i;
      if (Mx > M)
        valid2 &= (1<<M)-1;
      size_t m = movemask(valid2);
      goto entry;
      while (true)
      {
        if (unlikely(m == 0))
          return false;
      entry:
        size_t i=bsf(m);

        const unsigned int geomID = tri.geomIDs[i];
        Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

        /* execute occlusion filer */
        if (filter) {
          if (unlikely(context->hasContextFilter() ||
                       geometry->hasOcclusionFilter()))
          {
            const Vec2f uv = Vec2f(hit.U[i], hit.V[i]);
            const float old_t = ray.tfar[k];
            ray.tfar[k] = hit.T[i];
            HitK<K> h(context->instID,
                      geomID, tri.primIDs[i],
                      uv.x, uv.y,
                      hit.Ng(i));
            if (any(runOcclusionFilter(vbool<K>(1<<k),
                                       geometry,
                                       ray,context,h)))
              return false;
            ray.tfar[k] = old_t;
            m = btc(m,i);
            continue;
          }
        }
        break;
      }
#endif
      return true;

    }

    template<int M, int Mx, int K, bool filter>
    static __forceinline void
    epilogKthRayMTrisIntersect(IntersectContext* context,
                               RayHitK<K>& ray,
                               size_t k,
                               const vbool<Mx>& valid_i,
                               MTHitM<M>& hit,
                               const TriangleM<M>& tri)
    {
      // Do the epilog
      Scene* scene = context->scene;
      vbool<Mx> valid2 = valid_i;
      if (Mx > M)
        valid2 &= (1<<M)-1;
      size_t i = select_min(valid2, hit.T);
      assert(i<M);
      unsigned int geomID = tri.geomIDs[i];

        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION)
      bool foundhit = false;
      goto entry;
      while (true)
      {
        if (unlikely(none(valid2)))
          return;
        i = select_min(valid2, hit.T);
        assert(i<M);
        geomID = tri.geomIDs[i];
      entry:
        Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

        /* call intersection filter function */
        if (filter) {
          if (unlikely(context->hasContextFilter() ||
                       geometry->hasIntersectionFilter())) {
            assert(i<M);
            const Vec2f uv = Vec2f(hit.U[i], hit.V[i]);
            HitK<K> h(context->instID,
                      geomID, tri.primIDs[i],
                      uv.x,uv.y,
                      hit.Ng(i));
            const float old_t = ray.tfar[k];
            ray.tfar[k] = hit.T[i];
            const bool found = any(runIntersectionFilter(vbool<K>(1<<k),geometry,ray,context,h));
            if (!found) ray.tfar[k] = old_t;
            foundhit = foundhit | found;
            clear(valid2, i);
            // intersection filters may modify tfar value
            valid2 &= hit.T <= ray.tfar[k];
            continue;
          }
        }
        break;
      }
#endif
      assert(i<M);
      /* update hit information */
      const Vec2f uv = Vec2f(hit.U[i], hit.V[i]);
      ray.tfar[k] = hit.T[i];
      ray.Ng.x[k] = hit.N.x[i];
      ray.Ng.y[k] = hit.N.y[i];
      ray.Ng.z[k] = hit.N.z[i];
      ray.u[k] = uv.x;
      ray.v[k] = uv.y;
      ray.primID[k] = tri.primIDs[i];
      ray.geomID[k] = geomID;
      ray.instID[k] = context->instID;
    }

    template<int M, int Mx>
    static __forceinline bool
    epilog1RayMTrisOccluded(Ray& ray,
                            IntersectContext* context,
                            const TriangleM<M>& tri,
                            MTHitM<M>& hit)
    {
      Scene* scene = context->scene;
      /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION)

      vbool<Mx> valid = hit.valid;
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

#if defined(EMBREE_FILTER_FUNCTION)
        /* if we have no filter then the test passed */
        if (unlikely(context->hasContextFilter() ||
                     geometry->hasOcclusionFilter())) {
          const Vec2f uv = Vec2f(hit.U[i], hit.V[i]);
          HitK<1> h(context->instID,
                    geomID,
                    tri.primIDs[i],
                    uv.x, uv.y,
                    hit.Ng(i));
          const float old_t = ray.tfar;
          ray.tfar = hit.T[i];
          if (runOcclusionFilter1(geometry,ray,context,h))
            return true;
          ray.tfar = old_t;
          m = btc(m,i);
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
    epilog1RayMTrisIntersect(RayHit& ray,
                             IntersectContext* context,
                             const TriangleM<M>& tri,
                             MTHitM<M>& hit)
    {
      Scene* scene = context->scene;
      vbool<Mx> valid = hit.valid;
      if (Mx > M)
        valid &= (1<<M)-1;
      size_t i = select_min(valid, hit.T);
      unsigned int geomID = tri.geomIDs[i];

      /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION)
      bool foundhit = false;
      goto entry;
      while (true)
      {
        if (unlikely(none(valid))) return foundhit;
        i = select_min(valid,hit.T);

        geomID = tri.geomIDs[i];
      entry:
        Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

        /* call intersection filter function */
        if (unlikely(context->hasContextFilter() || geometry->hasIntersectionFilter())) {
          const Vec2f uv = Vec2f(hit.U[i], hit.V[i]);
          HitK<1> h(context->instID,
                    geomID, tri.primIDs[i],
                    uv.x,uv.y,
                    hit.Ng(i));
          const float old_t = ray.tfar;
          ray.tfar = hit.T[i];
          const bool found = runIntersectionFilter1(geometry,
                                                    ray,
                                                    context,h);
          if (!found) ray.tfar = old_t;
          foundhit |= found;
          clear(valid,i);
          // intersection filters may modify tfar value
          valid &= hit.T <= ray.tfar;
          continue;
        }
        break;
      }
#endif

      /* update hit information */
      const Vec2f uv = Vec2f(hit.U[i], hit.V[i]);
      ray.tfar = hit.T[i];
      ray.Ng.x = hit.N.x[i];
      ray.Ng.y = hit.N.y[i];
      ray.Ng.z = hit.N.z[i];
      ray.u = uv.x;
      ray.v = uv.y;
      ray.primID = tri.primIDs[i];
      ray.geomID = geomID;
      ray.instID = context->instID;
      return true;
    }

    /*! Intersects M triangles with 1 ray */
    // Note: Need to check again that filter is always true.
    template<int M, int Mx>
    struct TriangleMIntersector1Moeller
    {
      typedef TriangleM<M> Primitive;

      /*! Intersect a ray with the M triangles and updates the hit. */
      static __forceinline void
      intersect(RayHit& ray,
                IntersectContext* context,
                const TriangleM<M>& tri)
      {
        STAT3(normal.trav_prims, 1, 1, 1);
        MTHitM<M> hit;
        if (likely(intersect1RayMTris<M>(ray, tri, hit))) {
          epilog1RayMTrisIntersect<M, Mx>(ray, context, tri, hit);
        }
      }

      /*! Test if the ray is occluded by one of M triangles. */
      static __forceinline bool
      occluded(Ray& ray,
               IntersectContext* context,
               const TriangleM<M>& tri)
      {
        //printf("TriangleMIntersector1Moeller::occluded\n");
        STAT3(shadow.trav_prims, 1, 1, 1);
        MTHitM<M> hit;
        if (likely(intersect1RayMTris<M>(ray, tri, hit))) {
          return epilog1RayMTrisOccluded<M, Mx>(ray, context, tri, hit);
        }
        return false;
      }
    };

    /*! Intersects M triangles with K rays. */
    template<int M, int Mx, int K, bool filter>
    struct TriangleMIntersectorKMoeller
    {
      typedef TriangleM<M> Primitive;

      /*! Intersects K rays with M triangles. */
      static __forceinline
      void intersect(const vbool<K>& valid_i,
                     RayHitK<K>& ray,
                     IntersectContext* context,
                     const TriangleM<M>& tri)
      {
        //printf("TriangleMIntersectorKMoeller::intersect %d (packet)\n", filter);
        STAT_USER(0, TriangleM<M>::max_size());
        for (size_t i = 0; i < TriangleM<M>::max_size(); i++)
        {
          if (tri.geomIDs[i] == -1)
            break;
          STAT3(normal.trav_prims, 1, popcnt(valid_i), K);
          MTHitK<K> hit;
          if (likely(intersectKRaysMTris(ray, i, valid_i, hit, tri))) {
            epilogKRaysMTrisIntersect<M,K,filter>(
              context, ray, i, hit.valid, hit, tri);
          }
        }
      }

      /* Intersect the kth ray with M triangles and updates the
      hit. */
      static __forceinline void
      intersect(RayHitK<K>& ray,
                size_t k,
                IntersectContext* context,
                const TriangleM<M>& tri)
      {
        //printf("TriangleMIntersectorKMoeller::intersect %d (one ray)\n", filter);
        STAT3(normal.trav_prims,1,1,1);
        MTHitM<M> hit;
        if (likely(intersectKthRayMTris(ray, k, tri, hit))) {
          epilogKthRayMTrisIntersect<M, Mx, K, filter>(
            context, ray, k, hit.valid, hit, tri);
        }
      }

      /*! Test for K rays if they are occluded by any of the M
      triangles. When is this code run? */
      static __forceinline vbool<K>
      occluded(const vbool<K>& valid_i,
               RayK<K>& ray,
               IntersectContext* context,
               const TriangleM<M>& tri)
      {
        //printf("TriangleMIntersectorKMoeller::occluded %d (packet)\n", filter);
        // For occlusion we make a copy...
        vbool<K> valid0 = valid_i;

        for (size_t i = 0; i < TriangleM<M>::max_size(); i++) {
          if (tri.geomIDs[i] == -1)
            break;
          STAT3(shadow.trav_prims,1,popcnt(valid0),K);

          MTHitK<K> hit;
          if (likely(intersectKRaysMTris(ray, i, valid0, hit, tri))) {
            epilogKRaysMTrisOccluded<M, K, filter>(
              context, ray, hit.valid, hit, tri, i);
          }
          if (none(valid0))
            break;
        }
        return !valid0;
      }

      /*! Test if the ray is occluded by one of the M triangles. */
      static __forceinline bool
      occluded(RayK<K>& ray,
               size_t k,
               IntersectContext* context,
               const TriangleM<M>& tri)
      {
        //printf("TriangleMIntersectorKMoeller::occluded %d (one ray)\n", filter);
        STAT3(shadow.trav_prims,1,1,1);
        MTHitM<M> hit;
        bool val = intersectKthRayMTris<M, K>(ray, k, tri, hit);
        if (likely(val)) {
          return epilogKthRayMTrisOccluded<M, Mx, K, filter>(
            context, ray, k, hit.valid, hit, tri);
        }
        return false;
      }
    };
  }
}
