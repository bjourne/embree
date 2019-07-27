#pragma once

#include "triangle.h"
#include "triangle_intersector_moeller.h"
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
    template<int M, int K>
    static __forceinline bool
    intersectKRaysMTris(const RayK<K>& ray,
                        size_t i,
                        const vbool<K>& valid0,
                        MTHitK<K>& hit,
                        const TriangleM<M>& tri)
    {
      Vec3vf<K> n0 = broadcast<vfloat<K>>(tri.n0, i);
      Vec3vf<K> n1 = broadcast<vfloat<K>>(tri.n1, i);
      Vec3vf<K> n2 = broadcast<vfloat<K>>(tri.n2, i);

      vfloat<K> d0 = vfloat<K>(tri.d0[i]);
      vfloat<K> d1 = vfloat<K>(tri.d1[i]);
      vfloat<K> d2 = vfloat<K>(tri.d2[i]);

      vfloat<K> det = dot(n0, ray.org);
      vfloat<K> dett = d0 - dot(ray.org, n0);
      Vec3vf<K> wr = ray.org * det + ray.dir * dett;

      vfloat<K> u = dot(wr, n1) + det * d1;
      vfloat<K> v = dot(wr, n2) + det * d2;
      vfloat<K> tmpdet0 = det - u - v;

      vfloat<K> pdet0 = (tmpdet0 ^ u) | (u ^ v);

      // Return if negative
      vfloat<K> rdet = rcp(det);
      u = u * det;
      v = v * det;
      vfloat<K> t = (dett * rdet) | signmsk(pdet0);

      vbool<K> valid = valid0;
      valid &= (ray.tnear() < t) & (t <= ray.tfar);
      if (likely(none(valid)))
        return false;
      return true;
    }
    #else


    template<int M, int K>
    static __forceinline bool
    intersectKRaysMTris(const RayK<K>& ray,
                        size_t i,
                        const vbool<K>& valid0,
                        MTHitK<K>& hit,
                        const TriangleM<M>& tri)
    {
      // Do broadcasting and cross prod.
      Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0, i);
      Vec3<vfloat<K>> e1 = broadcast<vfloat<K>>(tri.e1, i);
      Vec3<vfloat<K>> e2 = broadcast<vfloat<K>>(tri.e2, i);
      Vec3<vfloat<K>> tri_Ng = cross(e2, e1);

      /* calculate denominator */
      vbool<K> valid = valid0;
      const Vec3vf<K> C = v0 - ray.org;
      const Vec3vf<K> R = cross(C, ray.dir);
      const vfloat<K> den = dot(tri_Ng, ray.dir);
      const vfloat<K> absDen = abs(den);
      const vfloat<K> sgnDen = signmsk(den);

      /* test against edge e2 v0 */
      const vfloat<K> U = dot(e2, R) ^ sgnDen;
      valid &= U >= 0.0f;
      if (likely(none(valid)))
        return false;

      /* test against edge v0 e1 */
      const vfloat<K> V = dot(e1, R) ^ sgnDen;
      valid &= V >= 0.0f;
      if (likely(none(valid)))
        return false;

      /* test against edge e1 e2 */
      const vfloat<K> W = absDen-U-V;
      valid &= W >= 0.0f;
      if (likely(none(valid)))
        return false;

      /* perform depth test */
      const vfloat<K> T = dot(tri_Ng,C) ^ sgnDen;
      valid &= (absDen*ray.tnear() < T) & (T <= absDen*ray.tfar);
      if (unlikely(none(valid)))
        return false;

      valid &= den != vfloat<K>(zero);
      if (unlikely(none(valid)))
        return false;

      /* calculate hit information */
      const vfloat<K> rcpAbsDen = rcp(absDen);
      new (&hit) MTHitK<K>(valid,
                           U * rcpAbsDen,
                           V * rcpAbsDen,
                           T * rcpAbsDen,
                           tri_Ng);
      return true;
    }
    #endif

    template<int M, int K>
    static __forceinline bool
    intersectKthRayMTris(const RayK<K>& ray,
                         size_t k,
                         const TriangleM<M>& tri,
                         MTHitM<M>& hit)
    {
      const Vec3vf<M> tri_Ng = cross(tri.e2, tri.e1);

      const Vec3vf<M> O = broadcast<vfloat<M>>(ray.org, k);
      const Vec3vf<M> D = broadcast<vfloat<M>>(ray.dir, k);
      const Vec3vf<M> C = Vec3vf<M>(tri.v0) - O;
      const Vec3vf<M> R = cross(C,D);
      const vfloat<M> den = dot(Vec3vf<M>(tri_Ng),D);
      const vfloat<M> absDen = abs(den);
      const vfloat<M> sgnDen = signmsk(den);

      /* perform edge tests */
      const vfloat<M> U = dot(Vec3vf<M>(tri.e2), R) ^ sgnDen;
      const vfloat<M> V = dot(Vec3vf<M>(tri.e1), R) ^ sgnDen;

      vbool<M> valid = (den != vfloat<M>(zero)) &
        (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
      if (likely(none(valid)))
        return false;

      /* perform depth test */
      const vfloat<M> T = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
      valid &= (absDen * vfloat<M>(ray.tnear()[k]) < T) &
        (T <= absDen * vfloat<M>(ray.tfar[k]));
      if (likely(none(valid)))
        return false;

      /* calculate hit information */
      const vfloat<M> rcpAbsDen = rcp(absDen);
      new (&hit) MTHitM<M>(valid,
                           U * rcpAbsDen,
                           V * rcpAbsDen,
                           T * rcpAbsDen,
                           tri_Ng);
      return true;
    }

    /*! Intersect 1 ray with one of M triangles. */
    template<int M>
    static __forceinline bool
    intersect1RayMTris(const vbool<M>& valid0,
                       Ray& ray,
                       const TriangleM<M>& tri,
                       MTHitM<M>& hit)
    {
      vbool<M> valid = valid0;
      const Vec3vf<M> O = Vec3vf<M>(ray.org);
      const Vec3vf<M> D = Vec3vf<M>(ray.dir);
      const Vec3vf<M> C = Vec3vf<M>(tri.v0) - O;
      const Vec3vf<M> R = cross(C,D);
      const Vec3vf<M> tri_Ng = cross(tri.e2, tri.e1);
      const vfloat<M> den = dot(Vec3vf<M>(tri_Ng), D);
      const vfloat<M> absDen = abs(den);
      const vfloat<M> sgnDen = signmsk(den);

      /* perform edge tests */
      const vfloat<M> U = dot(R, Vec3vf<M>(tri.e2)) ^ sgnDen;
      const vfloat<M> V = dot(R, Vec3vf<M>(tri.e1)) ^ sgnDen;

        /* perform backface culling */
      valid &= (den != vfloat<M>(zero)) &
        (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
      if (likely(none(valid)))
        return false;

      /* perform depth test */
      const vfloat<M> T = dot(Vec3vf<M>(tri_Ng),C) ^ sgnDen;
      valid &= (absDen*vfloat<M>(ray.tnear()) < T) &
        (T <= absDen*vfloat<M>(ray.tfar));
      if (likely(none(valid)))
        return false;

      /* update hit information */
      const vfloat<M> rcpAbsDen = rcp(absDen);
      new (&hit) MTHitM<M>(valid,
                           U * rcpAbsDen,
                           V * rcpAbsDen,
                           T * rcpAbsDen,
                           tri_Ng);
      return true;
    }

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
      typedef MoellerTrumboreIntersector1<Mx> Precalculations;

      /*! Intersect a ray with the M triangles and updates the hit. */
      static __forceinline void
      intersect(const MoellerTrumboreIntersector1<Mx>& pre,
                RayHit& ray,
                IntersectContext* context,
                const TriangleM<M>& tri)
      {
        STAT3(normal.trav_prims, 1, 1, 1);
        MTHitM<M> hit;
        vbool<M> valid = true;
        if (likely(intersect1RayMTris<M>(valid, ray, tri, hit))) {
          epilog1RayMTrisIntersect<M, Mx>(ray, context, tri, hit);
        }
      }

      /*! Test if the ray is occluded by one of M triangles. */
      static __forceinline bool
      occluded(const MoellerTrumboreIntersector1<Mx>& pre,
               Ray& ray,
               IntersectContext* context,
               const TriangleM<M>& tri)
      {
        //printf("TriangleMIntersector1Moeller::occluded\n");
        STAT3(shadow.trav_prims, 1, 1, 1);
        MTHitM<M> hit;
        vbool<M> valid = true;
        if (likely(intersect1RayMTris<M>(valid, ray, tri, hit))) {
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
      typedef MoellerTrumboreIntersectorK<Mx,K> Precalculations;

      /*! Intersects K rays with M triangles. */
      static __forceinline
      void intersect(const vbool<K>& valid_i,
                     MoellerTrumboreIntersectorK<Mx,K>& pre,
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
      intersect(MoellerTrumboreIntersectorK<Mx,K>& pre,
                RayHitK<K>& ray,
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
               Precalculations& pre,
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
      occluded(MoellerTrumboreIntersectorK<Mx,K>& pre,
               RayK<K>& ray,
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
