#pragma once

#include "bvh_factory.h"

namespace embree
{
/*! BVH4 instantiations */
class BVH4Factory : public BVHFactory
{
public:
    BVH4Factory(int bfeatures, int ifeatures);

public:
    Accel* BVH4Triangle4   (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

private:
    Accel::Intersectors
    BVH4Triangle4Intersectors(BVH4* bvh, IntersectVariant ivariant);

    static void
    createTriangleMeshTriangle4(TriangleMesh* mesh,
                                AccelData*& accel,
                                Builder*& builder);

private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
    // SAH scene builders
private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    // spatial scene builder
private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

    // twolevel scene builders
private:

    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);

};
}
