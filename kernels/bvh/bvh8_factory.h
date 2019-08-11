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
  /*! BVH8 instantiations */
  class BVH8Factory : public BVHFactory
  {
  public:
    BVH8Factory(int bfeatures, int ifeatures);

  public:
    Accel* BVH8Triangle4   (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4vMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH8Quad4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Quad4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Quad4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH8QuantizedTriangle4i(Scene* scene);
    Accel* BVH8QuantizedTriangle4(Scene* scene);
    Accel* BVH8QuantizedQuad4i(Scene* scene);

    Accel* BVH8UserGeometry(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);
    Accel* BVH8UserGeometryMB(Scene* scene);

    Accel* BVH8Instance(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);
    Accel* BVH8InstanceMB(Scene* scene);

    Accel* BVH8Grid(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8GridMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    static void createTriangleMeshTriangle4 (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);

  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

  private:
    Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Triangle4vIntersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Triangle4iIntersectors(BVH8* bvh, IntersectVariant ivariant);

    Accel::Intersectors QBVH8Triangle4iIntersectors(BVH8* bvh);
    Accel::Intersectors QBVH8Triangle4Intersectors(BVH8* bvh);
    Accel::Intersectors QBVH8Quad4iIntersectors(BVH8* bvh);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4iIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4Intersector1Moeller);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8IntersectorStreamPacketFallback);

    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4iIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4vIntersectorStreamPluecker);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4iIntersectorStreamPluecker);

    // SAH scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);

  };
}
