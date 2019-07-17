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

#include "trianglei.h"
#include "triangle_intersector_moeller.h"
#include "triangle_intersector_pluecker.h"

namespace embree
{
  namespace isa
  {
    /*! Intersects M triangles with 1 ray */
    template<int M, int Mx, bool filter>
    struct TriangleMiIntersector1Moeller
    {
      typedef TriangleMi<M> Primitive;
      typedef MoellerTrumboreIntersector1<Mx> Precalculations;

      static __forceinline void intersect(const Precalculations& pre,
                                          RayHit& ray,
                                          IntersectContext* context,
                                          const Primitive& tri)
      {
      }

      static __forceinline bool occluded(const Precalculations& pre,
                                         Ray& ray,
                                         IntersectContext* context,
                                         const Primitive& tri)
      {
        return true;
      }
    };

    /*! Intersects M triangles with K rays */
    template<int M, int Mx, int K, bool filter>
    struct TriangleMiIntersectorKMoeller
    {
      typedef TriangleMi<M> Primitive;
      typedef MoellerTrumboreIntersectorK<Mx,K> Precalculations;

      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const Primitive& tri)
      {
      }

      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& tri)
      {
        vbool<K> valid0 = valid_i;
        return !valid0;
      }

      static __forceinline void intersect(Precalculations& pre, RayHitK<K>& ray, size_t k, IntersectContext* context, const Primitive& tri)
      {
      }

      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive& tri)
      {
        return true;
      }
    };

    /*! Intersects M triangles with 1 ray */
    template<int M, int Mx, bool filter>
    struct TriangleMiIntersector1Pluecker
    {
      typedef TriangleMi<M> Primitive;
      typedef PlueckerIntersector1<Mx> Precalculations;

      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& tri)
      {
      }

      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& tri)
      {
        return true;
      }
    };

    /*! Intersects M triangles with K rays */
    template<int M, int Mx, int K, bool filter>
    struct TriangleMiIntersectorKPluecker
    {
      typedef TriangleMi<M> Primitive;
      typedef PlueckerIntersectorK<Mx,K> Precalculations;

      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const Primitive& tri)
      {
      }

      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& tri)
      {
        vbool<K> valid0 = valid_i;
        return !valid0;
      }

      static __forceinline void intersect(Precalculations& pre, RayHitK<K>& ray, size_t k, IntersectContext* context, const Primitive& tri)
      {
      }

      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive& tri)
      {
        return true;
      }
    };

    /*! Intersects M motion blur triangles with 1 ray */
    template<int M, int Mx, bool filter>
    struct TriangleMiMBIntersector1Moeller
    {
      typedef TriangleMi<M> Primitive;
      typedef MoellerTrumboreIntersector1<Mx> Precalculations;

      /*! Intersect a ray with the M triangles and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& tri)
      {
      }

      /*! Test if the ray is occluded by one of M triangles. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& tri)
      {
        return true;
      }
    };

    /*! Intersects M motion blur triangles with K rays. */
    template<int M, int Mx, int K, bool filter>
    struct TriangleMiMBIntersectorKMoeller
    {
      typedef TriangleMi<M> Primitive;
      typedef MoellerTrumboreIntersectorK<Mx,K> Precalculations;

      /*! Intersects K rays with M triangles. */
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const TriangleMi<M>& tri)
      {
      }

      /*! Test for K rays if they are occluded by any of the M triangles. */
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const TriangleMi<M>& tri)
      {
        vbool<K> valid0 = valid_i;
        return !valid0;
      }

      /*! Intersect a ray with M triangles and updates the hit. */
      static __forceinline void intersect(Precalculations& pre, RayHitK<K>& ray, size_t k, IntersectContext* context, const TriangleMi<M>& tri)
      {
      }

      /*! Test if the ray is occluded by one of the M triangles. */
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const TriangleMi<M>& tri)
      {
        return true;
      }
    };

    /*! Intersects M motion blur triangles with 1 ray */
    template<int M, int Mx, bool filter>
    struct TriangleMiMBIntersector1Pluecker
    {
      typedef TriangleMi<M> Primitive;
      typedef PlueckerIntersector1<Mx> Precalculations;

      /*! Intersect a ray with the M triangles and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& tri)
      {
      }

      /*! Test if the ray is occluded by one of M triangles. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& tri)
      {
        return true;
      }
    };

    /*! Intersects M motion blur triangles with K rays. */
    template<int M, int Mx, int K, bool filter>
    struct TriangleMiMBIntersectorKPluecker
    {
      typedef TriangleMi<M> Primitive;
      typedef PlueckerIntersectorK<Mx,K> Precalculations;

      /*! Intersects K rays with M triangles. */
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const TriangleMi<M>& tri)
      {
      }

      /*! Test for K rays if they are occluded by any of the M triangles. */
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const TriangleMi<M>& tri)
      {
        vbool<K> valid0 = valid_i;
        return !valid0;
      }

      /*! Intersect a ray with M triangles and updates the hit. */
      static __forceinline void intersect(Precalculations& pre, RayHitK<K>& ray, size_t k, IntersectContext* context, const TriangleMi<M>& tri)
      {
      }

      /*! Test if the ray is occluded by one of the M triangles. */
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const TriangleMi<M>& tri)
      {
        return true;
      }
    };
  }
}
