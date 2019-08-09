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

#include "bvh4_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/triangle.h"
#include "../common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Triangle4IntersectorStreamMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Triangle4IntersectorStreamMoellerNoFilter);

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

  void BVH4Factory::selectIntersectors(int features)
  {
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector1Moeller));

#if defined (EMBREE_RAY_PACKETS)

    /* select intersectors4 */
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512SKX(features,BVH4Triangle4Intersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512SKX(features,BVH4Triangle4Intersector4HybridMoellerNoFilter));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH4Triangle4Intersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH4Triangle4Intersector8HybridMoellerNoFilter));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector16HybridMoellerNoFilter));

    /* select stream intersectors */
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH4Triangle4IntersectorStreamMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH4Triangle4IntersectorStreamMoellerNoFilter));

#endif
  }

  Accel::Intersectors
  BVH4Factory::BVH4Triangle4Intersectors(BVH4* bvh, IntersectVariant ivariant)
  {
    assert(ivariant == IntersectVariant::FAST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           =
      BVH4Triangle4Intersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4_filter =
      BVH4Triangle4Intersector4HybridMoeller();
    intersectors.intersector4_nofilter =
      BVH4Triangle4Intersector4HybridMoellerNoFilter();
    intersectors.intersector8_filter    =
      BVH4Triangle4Intersector8HybridMoeller();
    intersectors.intersector8_nofilter  =
      BVH4Triangle4Intersector8HybridMoellerNoFilter();
    intersectors.intersector16_filter   =
      BVH4Triangle4Intersector16HybridMoeller();
    intersectors.intersector16_nofilter =
      BVH4Triangle4Intersector16HybridMoellerNoFilter();
    intersectors.intersectorN_filter    =
      BVH4Triangle4IntersectorStreamMoeller();
    intersectors.intersectorN_nofilter  =
      BVH4Triangle4IntersectorStreamMoellerNoFilter();
#endif
    return intersectors;
  }

  void
  BVH4Factory::createTriangleMeshTriangle4(TriangleMesh* mesh,
                                           AccelData*& accel,
                                           Builder*& builder)
  {
    BVH4Factory* factory = mesh->scene->device->bvh4_factory.get();
    accel = new BVH4(mesh->scene);
    builder = factory->BVH4Triangle4MeshBuilderSAH(accel,mesh,0);
  }

  Accel*
  BVH4Factory::BVH4Triangle4(Scene* scene,
                             BuildVariant bvariant,
                             IntersectVariant ivariant)
  {
    BVH4* accel = new BVH4(scene);

    Accel::Intersectors intersectors;
    if (scene->device->tri_traverser == "default")
      intersectors = BVH4Triangle4Intersectors(accel, ivariant);
    else if (scene->device->tri_traverser == "fast"   )
      intersectors = BVH4Triangle4Intersectors(accel,
                                               IntersectVariant::FAST);
    else
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4>");

    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default") {
      // Only static builders supported
      builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0);
    }
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4>");
    return new AccelInstance(accel,builder,intersectors);
  }

}
