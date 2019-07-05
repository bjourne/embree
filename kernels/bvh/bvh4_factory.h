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
    Accel* BVH4OBBVirtualCurve4i(Scene* scene);
    Accel* BVH4OBBVirtualCurve4v(Scene* scene);
    Accel* BVH4OBBVirtualCurve8i(Scene* scene);
    DEFINE_SYMBOL2(VirtualCurveIntersector*,VirtualCurveIntersector4i);
    DEFINE_SYMBOL2(VirtualCurveIntersector*,VirtualCurveIntersector8i);
    DEFINE_SYMBOL2(VirtualCurveIntersector*,VirtualCurveIntersector4v);
    DEFINE_SYMBOL2(VirtualCurveIntersector*,VirtualCurveIntersector8v);

    Accel* BVH4Triangle4   (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Triangle4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::ROBUST);
    Accel* BVH4Triangle4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH4Quad4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Quad4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH4QuantizedTriangle4i(Scene* scene);
    Accel* BVH4QuantizedQuad4i(Scene* scene);

    Accel* BVH4SubdivPatch1(Scene* scene);
    Accel* BVH4SubdivPatch1MB(Scene* scene);

    Accel* BVH4UserGeometry(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);
    Accel* BVH4UserGeometryMB(Scene* scene);

    Accel* BVH4Instance(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);
    Accel* BVH4InstanceMB(Scene* scene);

    Accel* BVH4Grid(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

private:
    Accel::Intersectors BVH4OBBVirtualCurveIntersectors(BVH4* bvh, VirtualCurveIntersector* leafIntersector);
      //Accel::Intersectors BVH4OBBVirtualCurveIntersectorsMB(BVH4* bvh, VirtualCurveIntersector* leafIntersector);

    Accel::Intersectors BVH4Triangle4Intersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Triangle4vIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Triangle4iIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Quad4vIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Quad4iIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors QBVH4Quad4iIntersectors(BVH4* bvh);
    Accel::Intersectors QBVH4Triangle4iIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4UserGeometryIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4InstanceIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4SubdivPatch1Intersectors(BVH4* bvh);
    Accel::Intersectors BVH4GridIntersectors(BVH4* bvh, IntersectVariant ivariant);

    static void createTriangleMeshTriangle4Morton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);

    static void createQuadMeshQuad4v(QuadMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createQuadMeshQuad4vMorton(QuadMesh* mesh, AccelData*& accel, Builder*& builder);

    static void createUserGeometryMesh(UserGeometry* mesh, AccelData*& accel, Builder*& builder);

private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,QBVH4Triangle4iIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,QBVH4Quad4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4SubdivPatch1Intersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4SubdivPatch1MBIntersector1);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4VirtualIntersector1);
    //DEFINE_SYMBOL2(Accel::Intersector1,BVH4VirtualMBIntersector1);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4InstanceIntersector1);
    //DEFINE_SYMBOL2(Accel::Intersector1,BVH4InstanceMBIntersector1);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4GridIntersector1Moeller);
    //DEFINE_SYMBOL2(Accel::Intersector1,BVH4GridMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4GridIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4SubdivPatch1Intersector4);
    //DEFINE_SYMBOL2(Accel::Intersector4,BVH4SubdivPatch1MBIntersector4);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4VirtualIntersector4Chunk);
    //DEFINE_SYMBOL2(Accel::Intersector4,BVH4VirtualMBIntersector4Chunk);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4InstanceIntersector4Chunk);
    //DEFINE_SYMBOL2(Accel::Intersector4,BVH4InstanceMBIntersector4Chunk);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4GridIntersector4HybridMoeller);
    //DEFINE_SYMBOL2(Accel::Intersector4,BVH4GridMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4GridIntersector4HybridPluecker);

    // ==============

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4OBBVirtualCurveIntersector8Hybrid);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4OBBVirtualCurveIntersector8HybridMB);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridPluecker);

    // DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vMBIntersector8HybridMoeller);
    // DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iMBIntersector8HybridMoeller);
    // DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vMBIntersector8HybridPluecker);
    // DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iMBIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iIntersector8HybridPluecker);

    // DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iMBIntersector8HybridMoeller);
    // DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iMBIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1Intersector8);
    //DEFINE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1MBIntersector8);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4VirtualIntersector8Chunk);
    //DEFINE_SYMBOL2(Accel::Intersector8,BVH4VirtualMBIntersector8Chunk);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4InstanceIntersector8Chunk);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4InstanceMBIntersector8Chunk);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4GridIntersector8HybridMoeller);
    //DEFINE_SYMBOL2(Accel::Intersector8,BVH4GridMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4GridIntersector8HybridPluecker);

    // SAH scene builders
private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Curve4vBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Curve4iBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Curve8iBuilder_OBB_New,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4QuantizedTriangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4QuantizedQuad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH4SubdivPatch1BuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4SubdivPatch1MBBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH4InstanceSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4InstanceMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH4GridSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4GridMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    // spatial scene builder
private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

    // twolevel scene builders
private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelQuadMeshSAH,void* COMMA Scene* COMMA const createQuadMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelVirtualSAH,void* COMMA Scene* COMMA const createUserGeometryAccelTy);

    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshBuilderSAH,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMeshBuilderSAH,void* COMMA UserGeometry* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4GridMeshBuilderSAH,void* COMMA GridMesh* COMMA size_t);

    // mesh refitters
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshRefitSAH,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMeshRefitSAH,void* COMMA UserGeometry* COMMA size_t);

    // morton mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t)
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshBuilderMortonGeneral,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMeshBuilderMortonGeneral,void* COMMA UserGeometry* COMMA size_t);
  };
}
