#include "bvh_intersector1.cpp"

namespace embree
{
namespace isa
{
/// BVH8Intersector1 Definitions

IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4Intersector1Moeller,  BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1Moeller  <SIMD_MODE(4) COMMA true> > >));


}
}
