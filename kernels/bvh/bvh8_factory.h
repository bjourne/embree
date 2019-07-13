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

    Accel* BVH8QuantizedTriangle4i(Scene* scene);
    Accel* BVH8QuantizedTriangle4(Scene* scene);

    Accel* BVH8Grid(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    static void createTriangleMeshTriangle4Morton (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4 (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);


  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

  private:
    Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors QBVH8Triangle4Intersectors(BVH8* bvh);
    Accel::Intersectors BVH8GridIntersectors(BVH8* bvh, IntersectVariant ivariant);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
      //DEFINE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4Intersector1Moeller);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
    // SAH scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

    // twolevel scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);

    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);

    // mesh refitters
  private:

    // morton mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  };
}
