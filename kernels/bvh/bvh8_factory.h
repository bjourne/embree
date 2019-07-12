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
    // Accel* BVH8OBBVirtualCurve8v(Scene* scene);

    Accel* BVH8Triangle4   (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH8Quad4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Quad4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH8QuantizedTriangle4i(Scene* scene);
    Accel* BVH8QuantizedTriangle4(Scene* scene);
    Accel* BVH8QuantizedQuad4i(Scene* scene);


      //Accel* BVH8Instance(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);

    Accel* BVH8Grid(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8GridMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    static void createTriangleMeshTriangle4Morton (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
      //static void createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
      //static void createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4 (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
      //static void createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);


  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

  private:
    // Accel::Intersectors BVH8OBBVirtualCurveIntersectors(BVH8* bvh, VirtualCurveIntersector* leafIntersector);
    // Accel::Intersectors BVH8OBBVirtualCurveIntersectorsMB(BVH8* bvh, VirtualCurveIntersector* leafIntersector);

    Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant);
      //Accel::Intersectors BVH8Triangle4iIntersectors(BVH8* bvh, IntersectVariant ivariant);

      //Accel::Intersectors QBVH8Triangle4iIntersectors(BVH8* bvh);
    Accel::Intersectors QBVH8Triangle4Intersectors(BVH8* bvh);

      //Accel::Intersectors BVH8InstanceIntersectors(BVH8* bvh);

    Accel::Intersectors BVH8GridIntersectors(BVH8* bvh, IntersectVariant ivariant);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);

    DEFINE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4Intersector1Moeller);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH8GridIntersector1Moeller);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
      //DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iIntersector4HybridMoeller);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8GridIntersector4HybridMoeller);


    // SAH scene builders
  private:
    // // DEFINE_ISA_FUNCTION(Builder*,BVH8Curve8vBuilder_OBB_New,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
      //DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8QuantizedQuad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

      //DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
      //DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH8InstanceSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
      //DEFINE_ISA_FUNCTION(Builder*,BVH8InstanceMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH8GridSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    // SAH spatial scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

    // twolevel scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelQuadMeshSAH,void* COMMA Scene* COMMA const createQuadMeshAccelTy);

    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
      //DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshBuilderSAH,void* COMMA QuadMesh* COMMA size_t);
      //DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualMeshBuilderSAH,void* COMMA UserGeometry* COMMA size_t);

    // mesh refitters
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8GridMeshBuilderSAH,void* COMMA GridMesh* COMMA size_t);

    // morton mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  };
}
