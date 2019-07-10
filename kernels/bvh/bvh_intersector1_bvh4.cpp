#include "bvh_intersector1.cpp"

namespace embree
{
namespace isa
{
int getISA() {
    return VerifyMultiTargetLinking::getISA();
}

/// BVH4Intersector1 Definitions

IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH4Triangle4Intersector1Moeller,  BVHNIntersector1<4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1Moeller  <SIMD_MODE(4) COMMA true> > >));


}
}
