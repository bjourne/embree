#include "bvh4_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/triangle.h"
#include "../geometry/subdivpatch1.h"
#include "../geometry/object.h"
#include "../geometry/instance.h"
#include "../geometry/subgrid.h"
#include "../common/accelinstance.h"

namespace embree
{

DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);

BVH4Factory::BVH4Factory(int bfeatures, int ifeatures)
{
    selectBuilders(bfeatures);
    selectIntersectors(ifeatures);
}

void BVH4Factory::selectBuilders(int features)
{
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4SceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4MeshBuilderSAH));
}

void
BVH4Factory::selectIntersectors(int features)
{
    printf("BVH4Factory::selectIntersectors\n");
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector1Moeller));
}

Accel::Intersectors
BVH4Factory::BVH4Triangle4Intersectors(BVH4* bvh, IntersectVariant ivariant)
{
    printf("BVH4Factory::BVH4Triangle4Intersectors\n");
    assert(ivariant == IntersectVariant::FAST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH4Triangle4Intersector1Moeller();
    return intersectors;
}

void
BVH4Factory::createTriangleMeshTriangle4(TriangleMesh* mesh,
                                         AccelData*& accel,
                                         Builder*& builder)
{
    printf("BVH4Factory::createTriangleMeshTriangle4 qual %d\n", mesh->quality);
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Triangle4::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH4Triangle4MeshBuilderSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
}

Accel* BVH4Factory::BVH4Triangle4(Scene* scene,
                                  BuildVariant bvariant,
                                  IntersectVariant ivariant)
{
    printf("BVH4Factory::BVH4Triangle4 %d\n", bvariant);
    BVH4* accel = new BVH4(Triangle4::type,scene);

    Accel::Intersectors intersectors;
    if (scene->device->tri_traverser == "default")
        intersectors = BVH4Triangle4Intersectors(accel, ivariant);
    else if (scene->device->tri_traverser == "fast"   )
        intersectors = BVH4Triangle4Intersectors(accel,IntersectVariant::FAST);
    else
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4>");

    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default") {
        switch (bvariant) {
        case BuildVariant::STATIC      : builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0); break;
        }
    }
    else if (scene->device->tri_builder == "sah"         )
        builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit")
        builder = BVH4Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
}

}
