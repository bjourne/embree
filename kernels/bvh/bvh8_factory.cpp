#include "../common/isa.h" // to define EMBREE_TARGET_SIMD8

#if defined (EMBREE_TARGET_SIMD8)

#include "bvh8_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/curveNv.h"
#include "../geometry/curveNi.h"
//#include "../geometry/curveNi_mb.h"
#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
//#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/subdivpatch1.h"
#include "../geometry/object.h"
#include "../geometry/instance.h"
#include "../geometry/subgrid.h"
#include "../common/accelinstance.h"

namespace embree
{
DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Moeller);
//DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vIntersector1Pluecker);
DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Pluecker);

DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4iIntersector1Pluecker);
DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4Intersector1Moeller);
DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Quad4iIntersector1Pluecker);

DECLARE_SYMBOL2(Accel::Intersector1,BVH8VirtualIntersector1);

DECLARE_SYMBOL2(Accel::Intersector1,BVH8InstanceIntersector1);

DECLARE_SYMBOL2(Accel::Intersector1,BVH8GridIntersector1Moeller);
DECLARE_SYMBOL2(Accel::Intersector1,BVH8GridIntersector1Pluecker);

DECLARE_SYMBOL2(Accel::Intersector4,BVH8VirtualIntersector4Chunk);

DECLARE_SYMBOL2(Accel::Intersector4,BVH8InstanceIntersector4Chunk);

DECLARE_SYMBOL2(Accel::Intersector4,BVH8GridIntersector4HybridMoeller);
DECLARE_SYMBOL2(Accel::Intersector4,BVH8GridIntersector4HybridPluecker);

//DECLARE_SYMBOL2(Accel::Intersector8,BVH8VirtualIntersector8Chunk);

DECLARE_SYMBOL2(Accel::Intersector8,BVH8InstanceIntersector8Chunk);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
//  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8QuantizedQuad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
//DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8InstanceSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
//DECLARE_ISA_FUNCTION(Builder*,BVH8InstanceMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8GridSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
//DECLARE_ISA_FUNCTION(Builder*,BVH8GridMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
  DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelQuadMeshSAH,void* COMMA Scene* COMMA const createQuadMeshAccelTy);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshBuilderSAH,void* COMMA QuadMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMeshBuilderSAH,void* COMMA UserGeometry* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8GridMeshBuilderSAH,void* COMMA GridMesh* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshRefitSAH,void* COMMA QuadMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMeshRefitSAH,void* COMMA UserGeometry* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshBuilderMortonGeneral,void* COMMA QuadMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMeshBuilderMortonGeneral,void* COMMA UserGeometry* COMMA size_t);

  BVH8Factory::BVH8Factory(int bfeatures, int ifeatures)
  {
    selectBuilders(bfeatures);
    selectIntersectors(ifeatures);
  }

void BVH8Factory::selectBuilders(int features)
{
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4iSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedTriangle4iSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedTriangle4SceneBuilderSAH));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4iSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedQuad4iSceneBuilderSAH));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX(features,BVH8VirtualSceneBuilderSAH));

    IF_ENABLED_INSTANCE(SELECT_SYMBOL_INIT_AVX(features,BVH8InstanceSceneBuilderSAH));

    IF_ENABLED_GRIDS(SELECT_SYMBOL_INIT_AVX(features,BVH8GridSceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderFastSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vSceneBuilderFastSpatialSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vSceneBuilderFastSpatialSAH));

    IF_ENABLED_TRIS  (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelTriangleMeshSAH));
    IF_ENABLED_QUADS (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelQuadMeshSAH));
    //IF_ENABLED_USER  (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelVirtualSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4MeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vMeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4iMeshBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vMeshBuilderSAH));
    IF_ENABLED_USER (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8VirtualMeshBuilderSAH));
    IF_ENABLED_GRIDS(SELECT_SYMBOL_INIT_AVX(features,BVH8GridMeshBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4MeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vMeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4iMeshRefitSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vMeshRefitSAH));
    IF_ENABLED_USER (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8VirtualMeshRefitSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4MeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4vMeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4iMeshBuilderMortonGeneral));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Quad4vMeshBuilderMortonGeneral));
    IF_ENABLED_USER (SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8VirtualMeshBuilderMortonGeneral));
  }

  void BVH8Factory::selectIntersectors(int features)
  {
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersector1Moeller));
    //IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4vIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersector1Pluecker));


    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Triangle4iIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Triangle4Intersector1Moeller));
    //IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Quad4iIntersector1Pluecker));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8VirtualIntersector1));

    IF_ENABLED_INSTANCE(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8InstanceIntersector1));

    // IF_ENABLED_GRIDS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8GridIntersector1Moeller));
    // IF_ENABLED_GRIDS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8GridIntersector1Pluecker));

#if defined (EMBREE_RAY_PACKETS)

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8VirtualIntersector4Chunk));
    IF_ENABLED_INSTANCE(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8InstanceIntersector4Chunk));
    //IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8VirtualIntersector8Chunk));
    IF_ENABLED_INSTANCE(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8InstanceIntersector8Chunk));
#endif
  }

  void BVH8Factory::createTriangleMeshTriangle4Morton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4::type,mesh->scene);
    builder = factory->BVH8Triangle4MeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH8Factory::createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4v::type,mesh->scene);
    builder = factory->BVH8Triangle4vMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH8Factory::createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4i::type,mesh->scene);
    builder = factory->BVH8Triangle4iMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH8Factory::createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH8Triangle4MeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH8Triangle4MeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH8Triangle4MeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

  void BVH8Factory::createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4v::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH8Triangle4vMeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH8Triangle4vMeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH8Triangle4vMeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

  void BVH8Factory::createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4i::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH8Triangle4iMeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH8Triangle4iMeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH8Triangle4iMeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

  void BVH8Factory::createUserGeometryMesh(UserGeometry* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Object::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH8VirtualMeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH8VirtualMeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH8VirtualMeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

Accel::Intersectors BVH8Factory::BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant)
{
    assert(ivariant == IntersectVariant::FAST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH8Triangle4Intersector1Moeller();
    return intersectors;
}

Accel::Intersectors BVH8Factory::BVH8Triangle4iIntersectors(BVH8* bvh, IntersectVariant ivariant)
{
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
        Accel::Intersectors intersectors;
        intersectors.ptr = bvh;
        intersectors.intersector1  = BVH8Triangle4iIntersector1Moeller();
        return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
        Accel::Intersectors intersectors;
        intersectors.ptr = bvh;
        intersectors.intersector1  = BVH8Triangle4iIntersector1Pluecker();
        return intersectors;
    }
    }
    return Accel::Intersectors();
}

  Accel::Intersectors BVH8Factory::QBVH8Triangle4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Triangle4iIntersector1Pluecker();
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::QBVH8Triangle4Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Triangle4Intersector1Moeller();
    return intersectors;
  }

  // Accel::Intersectors BVH8Factory::QBVH8Quad4iIntersectors(BVH8* bvh)
  // {
  //   Accel::Intersectors intersectors;
  //   intersectors.ptr = bvh;
  //   intersectors.intersector1 = QBVH8Quad4iIntersector1Pluecker();
  //   return intersectors;
  // }

  // Accel::Intersectors BVH8Factory::BVH8UserGeometryIntersectors(BVH8* bvh)
  // {
  //   Accel::Intersectors intersectors;
  //   intersectors.ptr = bvh;
  //   intersectors.intersector1  = BVH8VirtualIntersector1();
  //   return intersectors;
  // }

  Accel::Intersectors BVH8Factory::BVH8InstanceIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8InstanceIntersector1();
    return intersectors;
  }

  Accel* BVH8Factory::BVH8Triangle4(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel,ivariant);
    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default")  {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4); break;
      case BuildVariant::HIGH_QUALITY: builder = BVH8Triangle4SceneBuilderFastSpatialSAH(accel,scene,0); break;
      }
    }
    else if (scene->device->tri_builder == "sah"         )  builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial")  builder = BVH8Triangle4SceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit")     builder = BVH8Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     ) builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4);
    else if (scene->device->tri_builder == "morton"     ) builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4Morton);
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

Accel* BVH8Factory::BVH8Triangle4i(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
{
    BVH8* accel = new BVH8(Triangle4i::type,scene);
    Accel::Intersectors intersectors = BVH8Triangle4iIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default") {
        switch (bvariant) {
        case BuildVariant::STATIC      : builder = BVH8Triangle4iSceneBuilderSAH(accel,scene,0); break;
        case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4i); break;
        case BuildVariant::HIGH_QUALITY: assert(false); break; // FIXME: implement
        }
    }
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4i>");

    return new AccelInstance(accel,builder,intersectors);
}

Accel* BVH8Factory::BVH8QuantizedTriangle4i(Scene* scene)
{
    BVH8* accel = new BVH8(Triangle4i::type,scene);
    Accel::Intersectors intersectors = QBVH8Triangle4iIntersectors(accel);
    Builder* builder = BVH8QuantizedTriangle4iSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
}

  Accel* BVH8Factory::BVH8QuantizedTriangle4(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors = QBVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8QuantizedTriangle4SceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  // Accel* BVH8Factory::BVH8UserGeometry(Scene* scene, BuildVariant bvariant)
  // {
  //   BVH8* accel = new BVH8(Object::type,scene);
  //   Accel::Intersectors intersectors = BVH8UserGeometryIntersectors(accel);

  //   Builder* builder = nullptr;
  //   if (scene->device->object_builder == "default") {
  //     switch (bvariant) {
  //     case BuildVariant::STATIC      : builder = BVH8VirtualSceneBuilderSAH(accel,scene,0); break;
  //         //case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelVirtualSAH(accel,scene,&createUserGeometryMesh); break;
  //     case BuildVariant::HIGH_QUALITY: assert(false); break;
  //     }
  //   }
  //   else if (scene->device->object_builder == "sah") builder = BVH8VirtualSceneBuilderSAH(accel,scene,0);
  //   //else if (scene->device->object_builder == "dynamic") builder = BVH8BuilderTwoLevelVirtualSAH(accel,scene,&createUserGeometryMesh);
  //   else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->object_builder+" for BVH8<Object>");

  //   return new AccelInstance(accel,builder,intersectors);
  // }

  Accel* BVH8Factory::BVH8Instance(Scene* scene, BuildVariant bvariant)
  {
    BVH8* accel = new BVH8(InstancePrimitive::type,scene);
    Accel::Intersectors intersectors = BVH8InstanceIntersectors(accel);
    Builder* builder = BVH8InstanceSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel::Intersectors BVH8Factory::BVH8GridIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    if (ivariant == IntersectVariant::FAST)
    {
      intersectors.intersector1  = BVH8GridIntersector1Moeller();
    }
    else /* if (ivariant == IntersectVariant::ROBUST) */
    {
      intersectors.intersector1  = BVH8GridIntersector1Pluecker();
    }
    return intersectors;
  }

  Accel* BVH8Factory::BVH8Grid(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(SubGridQBVH8::type,scene);
    Accel::Intersectors intersectors = BVH8GridIntersectors(accel,ivariant);
    Builder* builder = nullptr;
    if (scene->device->grid_builder == "default") {
      builder = BVH8GridSceneBuilderSAH(accel,scene,0);
    }
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->object_builder+" for BVH4<GridMesh>");

    return new AccelInstance(accel,builder,intersectors);
  }
}

#endif
