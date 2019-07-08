#include "bvh4_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/curveNv.h"
#include "../geometry/curveNi.h"
#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
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

DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Moeller);
DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vIntersector1Pluecker);
DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Pluecker);

DECLARE_SYMBOL2(Accel::Intersector1,QBVH4Triangle4iIntersector1Pluecker);
DECLARE_SYMBOL2(Accel::Intersector1,QBVH4Quad4iIntersector1Pluecker);

DECLARE_SYMBOL2(Accel::Intersector1,BVH4VirtualIntersector1);

DECLARE_SYMBOL2(Accel::Intersector1,BVH4InstanceIntersector1);

DECLARE_SYMBOL2(Accel::Intersector1,BVH4GridIntersector1Moeller);
DECLARE_SYMBOL2(Accel::Intersector1,BVH4GridIntersector1Pluecker);

//DECLARE_SYMBOL2(Accel::Intersector4,BVH4VirtualIntersector4Chunk);

DECLARE_SYMBOL2(Accel::Intersector4,BVH4InstanceIntersector4Chunk);

//DECLARE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1Intersector8);

DECLARE_SYMBOL2(Accel::Intersector8,BVH4VirtualIntersector8Chunk);

DECLARE_SYMBOL2(Accel::Intersector8,BVH4InstanceIntersector8Chunk);

DECLARE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
DECLARE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelQuadMeshSAH,void* COMMA Scene* COMMA const createQuadMeshAccelTy);

DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4QuantizedTriangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4Quad4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Quad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4QuantizedQuad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4iSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4Quad4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshBuilderSAH,void* COMMA QuadMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4GridMeshBuilderSAH,void* COMMA GridMesh* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4VirtualSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4VirtualMeshBuilderSAH,void* COMMA UserGeometry* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4InstanceSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4GridSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4SubdivPatch1BuilderSAH,void* COMMA Scene* COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshRefitSAH,void* COMMA QuadMesh    * COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4VirtualMeshRefitSAH,void* COMMA UserGeometry    * COMMA size_t);

DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshBuilderMortonGeneral,void* COMMA QuadMesh    * COMMA size_t);
DECLARE_ISA_FUNCTION(Builder*,BVH4VirtualMeshBuilderMortonGeneral,void* COMMA UserGeometry    * COMMA size_t);

  BVH4Factory::BVH4Factory(int bfeatures, int ifeatures)
  {
    selectBuilders(bfeatures);
    selectIntersectors(ifeatures);
  }

void BVH4Factory::selectBuilders(int features)
{
    IF_ENABLED_TRIS (SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4BuilderTwoLevelTriangleMeshSAH));
    IF_ENABLED_QUADS (SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4BuilderTwoLevelQuadMeshSAH));
    //IF_ENABLED_USER (SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4BuilderTwoLevelVirtualSAH));

    // IF_ENABLED_CURVES(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Curve4vBuilder_OBB_New));
    // IF_ENABLED_CURVES(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Curve4iBuilder_OBB_New));
    // IF_ENABLED_CURVES(SELECT_SYMBOL_INIT_AVX(features,BVH4Curve8iBuilder_OBB_New));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4SceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4vSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4iSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4QuantizedTriangle4iSceneBuilderSAH));

    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Quad4vSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Quad4iSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4QuantizedQuad4iSceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4SceneBuilderFastSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vSceneBuilderFastSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iSceneBuilderFastSpatialSAH));

    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Quad4vSceneBuilderFastSpatialSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4MeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4vMeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4iMeshBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Quad4vMeshBuilderSAH));
    IF_ENABLED_GRIDS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4GridMeshBuilderSAH));

    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4VirtualSceneBuilderSAH));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4VirtualMeshBuilderSAH));

    IF_ENABLED_INSTANCE(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4InstanceSceneBuilderSAH));

    IF_ENABLED_GRIDS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4GridSceneBuilderSAH));

    IF_ENABLED_SUBDIV(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4SubdivPatch1BuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4MeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4vMeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4iMeshRefitSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Quad4vMeshRefitSAH));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4VirtualMeshRefitSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4MeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4vMeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Triangle4iMeshBuilderMortonGeneral));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4Quad4vMeshBuilderMortonGeneral));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,BVH4VirtualMeshBuilderMortonGeneral));
}

void BVH4Factory::selectIntersectors(int features)
{
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX512SKX(features,BVH4Triangle4iIntersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX512SKX(features,BVH4Triangle4vIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX512SKX(features,BVH4Triangle4iIntersector1Pluecker));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX512SKX(features,QBVH4Triangle4iIntersector1Pluecker));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512SKX(features,BVH4VirtualIntersector1));
    IF_ENABLED_INSTANCE(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512SKX(features,BVH4InstanceIntersector1));

#if defined (EMBREE_RAY_PACKETS)

    //IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512SKX(features,BVH4VirtualIntersector4Chunk));
    IF_ENABLED_INSTANCE(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512SKX(features,BVH4InstanceIntersector4Chunk));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH4VirtualIntersector8Chunk));
    IF_ENABLED_INSTANCE(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH4InstanceIntersector8Chunk));

#endif
}

Accel::Intersectors BVH4Factory::BVH4Triangle4Intersectors(BVH4* bvh, IntersectVariant ivariant)
{
    printf("BVH4Factory::BVH4Triangle4Intersectors\n");
    assert(ivariant == IntersectVariant::FAST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH4Triangle4Intersector1Moeller();
    return intersectors;
}

Accel::Intersectors BVH4Factory::BVH4Triangle4vIntersectors(BVH4* bvh, IntersectVariant ivariant)
{
    assert(ivariant == IntersectVariant::ROBUST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Triangle4vIntersector1Pluecker();
    return intersectors;
}

  Accel::Intersectors BVH4Factory::BVH4Triangle4iIntersectors(BVH4* bvh, IntersectVariant ivariant)
  {
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH4Triangle4iIntersector1Moeller();
      return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH4Triangle4iIntersector1Pluecker();
      return intersectors;
    }
    }
    return Accel::Intersectors();
  }

  Accel::Intersectors BVH4Factory::QBVH4Triangle4iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH4Triangle4iIntersector1Pluecker();
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4InstanceIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4InstanceIntersector1();
    return intersectors;
  }

void BVH4Factory::createTriangleMeshTriangle4Morton(TriangleMesh* mesh,
                                                    AccelData*& accel,
                                                    Builder*& builder)
{
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Triangle4::type,mesh->scene);
    builder = factory->BVH4Triangle4MeshBuilderMortonGeneral(accel,mesh,0);
}

  void BVH4Factory::createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Triangle4v::type,mesh->scene);
    builder = factory->BVH4Triangle4vMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH4Factory::createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Triangle4i::type,mesh->scene);
    builder = factory->BVH4Triangle4iMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH4Factory::createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Triangle4::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH4Triangle4MeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH4Triangle4MeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH4Triangle4MeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

  void BVH4Factory::createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Triangle4v::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH4Triangle4vMeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH4Triangle4vMeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH4Triangle4vMeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

  void BVH4Factory::createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Triangle4i::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH4Triangle4iMeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH4Triangle4iMeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH4Triangle4iMeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

void BVH4Factory::createUserGeometryMesh(UserGeometry* mesh,
                                         AccelData*& accel,
                                         Builder*& builder)
{
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(Object::type,mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_LOW:    builder = factory->BVH4VirtualMeshBuilderMortonGeneral(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:   builder = factory->BVH4VirtualMeshBuilderSAH(accel,mesh,0); break;
    case RTC_BUILD_QUALITY_REFIT:  builder = factory->BVH4VirtualMeshRefitSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
}

Accel* BVH4Factory::BVH4Triangle4(Scene* scene,
                                  BuildVariant bvariant,
                                  IntersectVariant ivariant)
{
    printf("BVH4Factory::BVH4Triangle4\n");

    BVH4* accel = new BVH4(Triangle4::type,scene);

    Accel::Intersectors intersectors;
    if (scene->device->tri_traverser == "default")
        intersectors = BVH4Triangle4Intersectors(accel,ivariant);
    else if (scene->device->tri_traverser == "fast"   )
        intersectors = BVH4Triangle4Intersectors(accel,IntersectVariant::FAST);
    else
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4>");

    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default") {
        switch (bvariant) {
        case BuildVariant::STATIC      : builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0); break;
        case BuildVariant::DYNAMIC     : builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4); break;
        case BuildVariant::HIGH_QUALITY: builder = BVH4Triangle4SceneBuilderFastSpatialSAH(accel,scene,0); break;
        }
    }
    else if (scene->device->tri_builder == "sah"         )
        builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial" )
        builder = BVH4Triangle4SceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit")
        builder = BVH4Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     )
        builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4);
    else if (scene->device->tri_builder == "morton"      )
        builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4Morton);
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
}

  Accel* BVH4Factory::BVH4Triangle4v(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH4* accel = new BVH4(Triangle4v::type,scene);

    Accel::Intersectors intersectors;
    if      (scene->device->tri_traverser == "default") intersectors = BVH4Triangle4vIntersectors(accel,ivariant);
    else if (scene->device->tri_traverser == "fast"   ) intersectors = BVH4Triangle4vIntersectors(accel,IntersectVariant::FAST);
    else if (scene->device->tri_traverser == "robust" ) intersectors = BVH4Triangle4vIntersectors(accel,IntersectVariant::ROBUST);
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4>");

    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default") {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH4Triangle4vSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4v); break;
      case BuildVariant::HIGH_QUALITY: builder = BVH4Triangle4vSceneBuilderFastSpatialSAH(accel,scene,0); break;
      }
    }
    else if (scene->device->tri_builder == "sah"         ) builder = BVH4Triangle4vSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial" ) builder = BVH4Triangle4vSceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH4Triangle4vSceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4v);
    else if (scene->device->tri_builder == "morton"      ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4vMorton);
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4v>");

    return new AccelInstance(accel,builder,intersectors);
  }

Accel* BVH4Factory::BVH4Triangle4i(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
{
    BVH4* accel = new BVH4(Triangle4i::type,scene);

    Accel::Intersectors intersectors;
    if      (scene->device->tri_traverser == "default") intersectors = BVH4Triangle4iIntersectors(accel,ivariant);
    else if (scene->device->tri_traverser == "fast"   ) intersectors = BVH4Triangle4iIntersectors(accel,IntersectVariant::FAST);
    else if (scene->device->tri_traverser == "robust" ) intersectors = BVH4Triangle4iIntersectors(accel,IntersectVariant::ROBUST);
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4i>");

    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default"     ) {
        switch (bvariant) {
        case BuildVariant::STATIC      : builder = BVH4Triangle4iSceneBuilderSAH(accel,scene,0); break;
        case BuildVariant::DYNAMIC     : builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4i); break;
        case BuildVariant::HIGH_QUALITY: builder = BVH4Triangle4iSceneBuilderFastSpatialSAH(accel,scene,0); break;
        }
    }
    else if (scene->device->tri_builder == "sah"         ) builder = BVH4Triangle4iSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial" ) builder = BVH4Triangle4iSceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH4Triangle4iSceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4i);
    else if (scene->device->tri_builder == "morton"      ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4iMorton);
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4i>");

    return new AccelInstance(accel,builder,intersectors);
}

  Accel* BVH4Factory::BVH4QuantizedTriangle4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4i::type,scene);
    Builder* builder = BVH4QuantizedTriangle4iSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = QBVH4Triangle4iIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

Accel* BVH4Factory::BVH4Instance(Scene* scene, BuildVariant bvariant)
{
    printf("Creating BVH4Instance\n");
    BVH4* accel = new BVH4(InstancePrimitive::type,scene);
    Accel::Intersectors intersectors = BVH4InstanceIntersectors(accel);
    Builder* builder = BVH4InstanceSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
}

Accel::Intersectors
BVH4Factory::BVH4GridIntersectors(BVH4* bvh,
                                  IntersectVariant ivariant)
{
    printf("BVH4Factory::BVH4GridIntersectors\n");
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    if (ivariant == IntersectVariant::FAST)
    {
        intersectors.intersector1  = BVH4GridIntersector1Moeller();
    }
    else /* if (ivariant == IntersectVariant::ROBUST) */
    {
        intersectors.intersector1  = BVH4GridIntersector1Pluecker();
    }
    return intersectors;
}

Accel* BVH4Factory::BVH4Grid(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
{
    BVH4* accel = new BVH4(SubGridQBVH4::type,scene);
    Accel::Intersectors intersectors = BVH4GridIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->object_builder == "default") {
      builder = BVH4GridSceneBuilderSAH(accel,scene,0);
    }
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->grid_builder+" for BVH4<GridMesh>");

    return new AccelInstance(accel,builder,intersectors);
}

}
