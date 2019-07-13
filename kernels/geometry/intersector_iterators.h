#pragma once

#include "../common/scene.h"
#include "../common/ray.h"
#include "../bvh/node_intersector1.h"
#include "../bvh/node_intersector_packet.h"

namespace embree
{
namespace isa
{
template<typename Intersector>
struct ArrayIntersector1
{
    typedef typename Intersector::Primitive Primitive;
    typedef typename Intersector::Precalculations Precalculations;

    template<int N, int Nx, bool robust>
    static __forceinline void
    intersect(const Accel::Intersectors* This,
              Precalculations& pre,
              RayHit& ray,
              IntersectContext* context,
              const Primitive* prim,
              size_t num,
              const TravRay<N,Nx,robust> &tray,
              size_t& lazy_node)
    {
        for (size_t i=0; i<num; i++)
            Intersector::intersect(ray,context,prim[i]);
    }

    template<int N, int Nx, bool robust>
    static __forceinline bool
    occluded(const Accel::Intersectors* This,
             Precalculations& pre,
             Ray& ray,
             IntersectContext* context,
             const Primitive* prim,
             size_t num,
             const TravRay<N,Nx,robust> &tray,
             size_t& lazy_node)
    {
        for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(ray,context,prim[i]))
                return true;
        }
        return false;
    }

    template<int K>
    static __forceinline void
    intersectK(const vbool<K>& valid,
               /* PrecalculationsK& pre, */
               RayHitK<K>& ray,
               IntersectContext* context,
               const Primitive* prim,
               size_t num,
               size_t& lazy_node)
    {
    }

    template<int K>
      static __forceinline vbool<K> occludedK(const vbool<K>& valid, /* PrecalculationsK& pre, */ RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t num, size_t& lazy_node)
      {
        return valid;
      }
    };

  }
}
