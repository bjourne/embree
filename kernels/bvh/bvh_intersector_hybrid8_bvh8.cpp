#include "bvh_intersector_hybrid.cpp"

namespace embree
{
namespace isa
{
////////////////////////////////////////////////////////////////////////////////
/// BVH8Intersector8 Definitions
////////////////////////////////////////////////////////////////////////////////


IF_ENABLED_CURVES(DEFINE_INTERSECTOR8(BVH8OBBVirtualCurveIntersector8Hybrid, BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1_UN1 COMMA false COMMA VirtualCurveIntersectorK<8> >));
IF_ENABLED_CURVES(DEFINE_INTERSECTOR8(BVH8OBBVirtualCurveIntersector8HybridMB, BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN2_AN4D_UN2 COMMA false COMMA VirtualCurveIntersectorK<8> >));

IF_ENABLED_USER(DEFINE_INTERSECTOR8(BVH8VirtualIntersector8Chunk, BVHNIntersectorKChunk<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA ObjectIntersector8> >));
IF_ENABLED_INSTANCE(DEFINE_INTERSECTOR8(BVH8InstanceIntersector8Chunk, BVHNIntersectorKChunk<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA InstanceIntersectorK<8>> >));

}
}
