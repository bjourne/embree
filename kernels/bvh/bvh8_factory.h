#pragma once

#include "bvh_factory.h"

namespace embree
{
/*! BVH8 instantiations */
class BVH8Factory : public BVHFactory
{
public:
    BVH8Factory(int bfeatures, int ifeatures);

public:
    Accel* BVH8Triangle4   (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    static void createTriangleMeshTriangle4 (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);


private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

  private:
    Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);

    // SAH scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  private:

    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
};
}
