// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../common/isa.h" // to define EMBREE_TARGET_SIMD8

#if defined (EMBREE_TARGET_SIMD8)

#include "bvh8_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/triangle.h"
#include "../common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);

  DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4Intersector1Moeller);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoellerNoFilter);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
  DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelVirtualSAH,void* COMMA Scene* COMMA const createUserGeometryAccelTy);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);

  BVH8Factory::BVH8Factory(int bfeatures, int ifeatures)
  {
    selectBuilders(bfeatures);
    selectIntersectors(ifeatures);
  }

  void BVH8Factory::selectBuilders(int features)
  {
    IF_ENABLED_CURVES(SELECT_SYMBOL_INIT_AVX(features,BVH8Curve8vBuilder_OBB_New));
    IF_ENABLED_CURVES(SELECT_SYMBOL_INIT_AVX(features,BVH8OBBCurve8iMBBuilder_OBB));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderFastSpatialSAH));

    IF_ENABLED_TRIS  (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelTriangleMeshSAH));
    IF_ENABLED_USER  (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelVirtualSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4MeshBuilderSAH));

    //IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4MeshRefitSAH));

  }

  void BVH8Factory::selectIntersectors(int features)
  {
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector1Moeller));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Triangle4Intersector1Moeller));

#if defined (EMBREE_RAY_PACKETS)

    /* select intersectors4 */
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector4HybridMoellerNoFilter));

    /* select intersectors8 */
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector8HybridMoellerNoFilter));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector16HybridMoellerNoFilter));

    /* select stream intersectors */
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4IntersectorStreamMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4IntersectorStreamMoellerNoFilter));

#endif
  }

  void BVH8Factory::createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    printf("BVH8Factory::createTriangleMeshTriangle4 "
           "mesh->quality = %d\n", mesh->quality);
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(mesh->scene);
    switch (mesh->quality) {
    case RTC_BUILD_QUALITY_MEDIUM:
    case RTC_BUILD_QUALITY_HIGH:
      builder = factory->BVH8Triangle4MeshBuilderSAH(accel,mesh,0); break;
    default:
      throw_RTCError(RTC_ERROR_UNKNOWN,"invalid build quality");
    }
  }

  Accel::Intersectors
  BVH8Factory::BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    printf("BVH8Factory::BVH8Triangle4Intersectors\n");
    assert(ivariant == IntersectVariant::FAST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle4Intersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4_filter    = BVH8Triangle4Intersector4HybridMoeller();
    intersectors.intersector4_nofilter  = BVH8Triangle4Intersector4HybridMoellerNoFilter();
    intersectors.intersector8_filter    = BVH8Triangle4Intersector8HybridMoeller();
    intersectors.intersector8_nofilter  = BVH8Triangle4Intersector8HybridMoellerNoFilter();
    intersectors.intersector16_filter   = BVH8Triangle4Intersector16HybridMoeller();
    intersectors.intersector16_nofilter = BVH8Triangle4Intersector16HybridMoellerNoFilter();
    intersectors.intersectorN_filter    = BVH8Triangle4IntersectorStreamMoeller();
    intersectors.intersectorN_nofilter  = BVH8Triangle4IntersectorStreamMoellerNoFilter();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::QBVH8Triangle4Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Triangle4Intersector1Moeller();
    return intersectors;
  }

  Accel*
  BVH8Factory::BVH8Triangle4(Scene* scene,
                             BuildVariant bvariant,
                             IntersectVariant ivariant)
  {
    printf("BVH8Factory::BVH8Triangle4 bvariant %d\n", bvariant);
    BVH8* accel = new BVH8(scene);
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
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

}

#endif
