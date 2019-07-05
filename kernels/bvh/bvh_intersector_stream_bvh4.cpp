#include "bvh_intersector_stream.cpp"

namespace embree
{
  namespace isa
  {

    ////////////////////////////////////////////////////////////////////////////////
    /// General BVHIntersectorStreamPacketFallback Intersector
    ////////////////////////////////////////////////////////////////////////////////

    DEFINE_INTERSECTORN(BVH4IntersectorStreamPacketFallback,BVHNIntersectorStreamPacketFallback<SIMD_MODE(4)>);

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4IntersectorStream Definitions
    ////////////////////////////////////////////////////////////////////////////////

    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4iIntersectorStreamMoeller,        BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA Triangle4iIntersectorStreamMoeller<true>>));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4vIntersectorStreamPluecker,       BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA true  COMMA Triangle4vIntersectorStreamPluecker<true>>));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4iIntersectorStreamPluecker,       BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA true  COMMA Triangle4iIntersectorStreamPluecker<true>>));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4IntersectorStreamMoeller,         BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA Triangle4IntersectorStreamMoeller<true>>));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4IntersectorStreamMoellerNoFilter, BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA Triangle4IntersectorStreamMoeller<false>>));

    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4vIntersectorStreamMoeller,        BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA Quad4vIntersectorStreamMoeller<true>>));
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4vIntersectorStreamMoellerNoFilter,BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA Quad4vIntersectorStreamMoeller<false>>));
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4iIntersectorStreamMoeller,        BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA Quad4iIntersectorStreamMoeller<true>>));
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4vIntersectorStreamPluecker,       BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA true  COMMA Quad4vIntersectorStreamPluecker<true>>));
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4iIntersectorStreamPluecker,       BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA true  COMMA Quad4iIntersectorStreamPluecker<true>>));

    IF_ENABLED_USER(DEFINE_INTERSECTORN(BVH4VirtualIntersectorStream,BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ObjectIntersectorStream>));
    IF_ENABLED_INSTANCE(DEFINE_INTERSECTORN(BVH4InstanceIntersectorStream,BVHNIntersectorStream<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA InstanceIntersectorStream>));
  }
}
