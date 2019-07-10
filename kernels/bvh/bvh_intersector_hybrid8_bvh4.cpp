#include "bvh_intersector_hybrid.cpp"

namespace embree
{
namespace isa
{
/// BVH4Intersector8 Definitions


//IF_ENABLED_CURVES(DEFINE_INTERSECTOR8(BVH4OBBVirtualCurveIntersector8Hybrid, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1_UN1 COMMA false COMMA VirtualCurveIntersectorK<8> >));

//IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR8(BVH4SubdivPatch1Intersector8, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1Intersector8>));

//IF_ENABLED_USER(DEFINE_INTERSECTOR8(BVH4VirtualIntersector8Chunk, BVHNIntersectorKChunk<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA ObjectIntersector8> >));

//IF_ENABLED_INSTANCE(DEFINE_INTERSECTOR8(BVH4InstanceIntersector8Chunk, BVHNIntersectorKChunk<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA InstanceIntersectorK<8>> >));


}


}
