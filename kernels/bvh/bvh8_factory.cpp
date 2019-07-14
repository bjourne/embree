#include "../common/isa.h" // to define EMBREE_TARGET_SIMD8

#if defined (EMBREE_TARGET_SIMD8)

#include "bvh8_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/triangle.h"
#include "../geometry/subdivpatch1.h"
#include "../geometry/object.h"
#include "../geometry/instance.h"
#include "../geometry/subgrid.h"
#include "../common/accelinstance.h"

namespace embree
{
DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
//DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);

BVH8Factory::BVH8Factory(int bfeatures, int ifeatures)
{
    selectBuilders(bfeatures);
    selectIntersectors(ifeatures);
}

void BVH8Factory::selectBuilders(int features)
{
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderSAH));
    //IF_ENABLED_TRIS  (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelTriangleMeshSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4MeshBuilderSAH));
}

void
BVH8Factory::selectIntersectors(int features)
{
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector1Moeller));
}

void
BVH8Factory::createTriangleMeshTriangle4(TriangleMesh* mesh,
                                         AccelData*& accel,
                                         Builder*& builder)
{
    printf("BVH8Factory::createTriangleMeshTriangle4\n");
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH8Triangle4MeshBuilderSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
}

Accel::Intersectors
BVH8Factory::BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant)
{
    assert(ivariant == IntersectVariant::FAST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH8Triangle4Intersector1Moeller();
    return intersectors;
}

Accel*
BVH8Factory::BVH8Triangle4(Scene* scene,
                           BuildVariant bvariant,
                           IntersectVariant ivariant)
{
    printf("BVH8Factory::BVH8Triangle4 %s\n",
           scene->device->tri_builder.c_str());
    BVH8* accel = new BVH8(Triangle4::type, scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel,ivariant);
    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default")  {
        switch (bvariant) {
        case BuildVariant::STATIC:
            builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
            break;
        // case BuildVariant::DYNAMIC:
        //     builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4);
            break;
        }
    }
    else
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

}

#endif
