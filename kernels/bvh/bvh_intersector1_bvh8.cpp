#include "bvh_intersector1.cpp"

namespace embree
{
namespace isa
{
/// BVH8Intersector1 Definitions

IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4Intersector1Moeller,  BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1Moeller  <SIMD_MODE(4) COMMA true> > >));
IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4iIntersector1Moeller, BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMiIntersector1Moeller <SIMD_MODE(4) COMMA true> > >));

IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(QBVH8Triangle4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<TriangleMiIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >));
IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(QBVH8Triangle4Intersector1Moeller,BVHNIntersector1<8 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1Moeller  <SIMD_MODE(4) COMMA true> > >));

IF_ENABLED_USER(DEFINE_INTERSECTOR1(BVH8VirtualIntersector1,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<ObjectIntersector1<false>> >));
IF_ENABLED_INSTANCE(DEFINE_INTERSECTOR1(BVH8InstanceIntersector1,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<InstanceIntersector1> >));


}
}
