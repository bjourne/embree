#include "bvh_intersector1.cpp"

namespace embree
{
namespace isa
{
/// BVH8Intersector1 Definitions

IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4Intersector1Moeller,  BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1Moeller  <SIMD_MODE(4) COMMA true> > >));
IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4iIntersector1Moeller, BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMiIntersector1Moeller <SIMD_MODE(4) COMMA true> > >));
IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4vIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersector1<TriangleMvIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >));
IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersector1<TriangleMiIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >));

IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4vIntersector1Woop,  BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMvIntersector1Woop  <4 COMMA 4 COMMA true> > >));

// IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH8Quad4vIntersector1Moeller, BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1Moeller <4 COMMA true> > >));
// IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH8Quad4iIntersector1Moeller, BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMiIntersector1Moeller <4 COMMA true> > >));
// IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH8Quad4vIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersector1<QuadMvIntersector1Pluecker<4 COMMA true> > >));
// IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH8Quad4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_AN1 COMMA true  COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >));

IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(QBVH8Triangle4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<TriangleMiIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >));
IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(QBVH8Triangle4Intersector1Moeller,BVHNIntersector1<8 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1Moeller  <SIMD_MODE(4) COMMA true> > >));

// IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(QBVH8Quad4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >));

IF_ENABLED_CURVES(DEFINE_INTERSECTOR1(BVH8OBBVirtualCurveIntersector1,BVHNIntersector1<8 COMMA BVH_AN1_UN1 COMMA false COMMA VirtualCurveIntersector1 >));

IF_ENABLED_USER(DEFINE_INTERSECTOR1(BVH8VirtualIntersector1,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<ObjectIntersector1<false>> >));

IF_ENABLED_INSTANCE(DEFINE_INTERSECTOR1(BVH8InstanceIntersector1,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<InstanceIntersector1> >));

// IF_ENABLED_GRIDS(DEFINE_INTERSECTOR1(BVH8GridIntersector1Moeller,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA SubGridIntersector1Moeller<8 COMMA true> >));

// IF_ENABLED_GRIDS(DEFINE_INTERSECTOR1(BVH8GridIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_AN1 COMMA true COMMA SubGridIntersector1Pluecker<8 COMMA true> >));

}
}
