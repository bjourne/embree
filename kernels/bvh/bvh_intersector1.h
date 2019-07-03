#pragma once

#include "bvh.h"
#include "../common/ray.h"

namespace embree
{
  namespace isa
  {
    /*! BVH single ray intersector. */
    template<int N, int types, bool robust, typename PrimitiveIntersector1>
    class BVHNIntersector1
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersector1::Precalculations Precalculations;
      typedef typename PrimitiveIntersector1::Primitive Primitive;
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::AlignedNode AlignedNode;
      typedef typename BVH::AlignedNodeMB4D AlignedNodeMB4D;

      static const size_t stackSize = 1+(N-1)*BVH::maxDepth+3; // +3 due to 16-wide store

      /* right now AVX512KNL SIMD extension only for standard node types */
      static const size_t Nx = (types == BVH_AN1 || types == BVH_QN1) ? vextend<N>::size : N;

    public:
      static void intersect(const Accel::Intersectors* This, RayHit& ray, IntersectContext* context);
      static void occluded (const Accel::Intersectors* This, Ray& ray, IntersectContext* context);
    };
  }
}
