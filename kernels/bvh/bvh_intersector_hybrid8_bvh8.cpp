#include "bvh_intersector_hybrid.cpp"

namespace embree
{
namespace isa
{
////////////////////////////////////////////////////////////////////////////////
/// BVH8Intersector8 Definitions
////////////////////////////////////////////////////////////////////////////////

IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8HybridMoeller,        BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 8 COMMA true> > >));
IF_ENABLED_TRIS(DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8HybridMoellerNoFilter,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoeller  <SIMD_MODE(4) COMMA 8 COMMA false> > >));

IF_ENABLED_CURVES(DEFINE_INTERSECTOR8(BVH8OBBVirtualCurveIntersector8Hybrid, BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1_UN1 COMMA false COMMA VirtualCurveIntersectorK<8> >));
IF_ENABLED_CURVES(DEFINE_INTERSECTOR8(BVH8OBBVirtualCurveIntersector8HybridMB, BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN2_AN4D_UN2 COMMA false COMMA VirtualCurveIntersectorK<8> >));

IF_ENABLED_USER(DEFINE_INTERSECTOR8(BVH8VirtualIntersector8Chunk, BVHNIntersectorKChunk<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA ObjectIntersector8> >));
IF_ENABLED_USER(DEFINE_INTERSECTOR8(BVH8VirtualMBIntersector8Chunk, BVHNIntersectorKChunk<8 COMMA 8 COMMA BVH_AN2_AN4D COMMA false COMMA ArrayIntersectorK_1<8 COMMA ObjectIntersector8MB> >));

IF_ENABLED_INSTANCE(DEFINE_INTERSECTOR8(BVH8InstanceIntersector8Chunk, BVHNIntersectorKChunk<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA InstanceIntersectorK<8>> >));

}
}
