#include "bvh_intersector_hybrid.cpp"

namespace embree
{
namespace isa
{
    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector4 Definitions
    ////////////////////////////////////////////////////////////////////////////////


    IF_ENABLED_CURVES(DEFINE_INTERSECTOR4(BVH8OBBVirtualCurveIntersector4Hybrid, BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1_UN1 COMMA false COMMA VirtualCurveIntersectorK<4> >));
    IF_ENABLED_CURVES(DEFINE_INTERSECTOR4(BVH8OBBVirtualCurveIntersector4HybridMB, BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN2_AN4D_UN2 COMMA false COMMA VirtualCurveIntersectorK<4> >));

    IF_ENABLED_USER(DEFINE_INTERSECTOR4(BVH8VirtualIntersector4Chunk, BVHNIntersectorKChunk<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA ObjectIntersector4> >));
    IF_ENABLED_USER(DEFINE_INTERSECTOR4(BVH8VirtualMBIntersector4Chunk, BVHNIntersectorKChunk<8 COMMA 4 COMMA BVH_AN2_AN4D COMMA false COMMA ArrayIntersectorK_1<4 COMMA ObjectIntersector4MB> >));

    IF_ENABLED_INSTANCE(DEFINE_INTERSECTOR4(BVH8InstanceIntersector4Chunk, BVHNIntersectorKChunk<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA InstanceIntersectorK<4>> >));
}
}
