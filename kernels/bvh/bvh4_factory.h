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
    Accel* BVH4Triangle4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::ROBUST);
    Accel* BVH4Triangle4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Triangle4vMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Triangle4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH4Quad4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Quad4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Quad4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

  private:
    Accel::Intersectors BVH4Triangle4Intersectors(BVH4* bvh, IntersectVariant ivariant);

    static void
    createTriangleMeshTriangle4(TriangleMesh* mesh,
                                AccelData*& accel,
                                Builder*& builder);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoellerNoFilter);

    // ==============

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridPluecker);

    // ==============

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iIntersector16HybridPluecker);

    // ==============

    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4IntersectorStreamPacketFallback);

    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4IntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4IntersectorStreamMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4iIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4vIntersectorStreamPluecker);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4iIntersectorStreamPluecker);

    // SAH scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    // spatial scene builder
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

    // twolevel scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelVirtualSAH,void* COMMA Scene* COMMA const createUserGeometryAccelTy);

    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);

  };
}
