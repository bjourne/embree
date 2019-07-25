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

#include "bvh.h"
#include "bvh_statistics.h"
#include "bvh_rotate.h"
#include "../common/profile.h"
#include "../../common/algorithms/parallel_prefix_sum.h"

#include "../builders/primrefgen.h"
#include "../builders/bvh_builder_morton.h"

#include "../geometry/triangle.h"

#if defined(__X86_64__)
#  define ROTATE_TREE 1 // specifies number of tree rotation rounds to perform
#else
#  define ROTATE_TREE 0 // do not use tree rotations on 32 bit platforms, barrier bit in NodeRef will cause issues
#endif

namespace embree
{
  namespace isa
  {
    template<int N>
    struct SetBVHNBounds
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecord NodeRecord;
      typedef typename BVH::AlignedNode AlignedNode;

      BVH* bvh;
      __forceinline SetBVHNBounds (BVH* bvh) : bvh(bvh) {}

      __forceinline NodeRecord operator() (NodeRef ref, const NodeRecord* children, size_t num)
      {
        AlignedNode* node = ref.alignedNode();

        BBox3fa res = empty;
        for (size_t i=0; i<num; i++) {
          const BBox3fa b = children[i].bounds;
          res.extend(b);
          node->setRef(i,children[i].ref);
          node->setBounds(i,b);
        }

#if ROTATE_TREE
        if (N == 4)
        {
          size_t n = 0;
          for (size_t i=0; i<num; i++)
            n += children[i].bounds.lower.a;

          if (n >= 4096) {
            for (size_t i=0; i<num; i++) {
              if (children[i].bounds.lower.a < 4096) {
                for (int j=0; j<ROTATE_TREE; j++)
                  BVHNRotate<N>::rotate(node->child(i));
                node->child(i).setBarrier();
              }
            }
          }
          res.lower.a = unsigned(n);
        }
#endif

        return NodeRecord(ref,res);
      }
    };

    template<int N, typename Primitive>
    struct CreateMortonLeaf;

    template<int N>
    struct CreateMortonLeaf<N,Triangle4>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecord NodeRecord;

      __forceinline CreateMortonLeaf (TriangleMesh* mesh, BVHBuilderMorton::BuildPrim* morton)
        : mesh(mesh), morton(morton) {}

      __noinline NodeRecord operator() (const range<unsigned>& current, const FastAllocator::CachedAllocator& alloc)
      {
        vfloat4 lower(pos_inf);
        vfloat4 upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin();
        assert(items<=4);

        /* allocate leaf node */
        Triangle4* accel = (Triangle4*) alloc.malloc1(sizeof(Triangle4),BVH::byteAlignment);
        NodeRef ref = BVH::encodeLeaf((char*)accel,1);
        vuint4 vgeomID = -1, vprimID = -1;
        Vec3vf4 v0 = zero, v1 = zero, v2 = zero;
        const unsigned int geomID = this->mesh->geomID;
        const TriangleMesh* __restrict__ const mesh = this->mesh;

        for (size_t i=0; i<items; i++)
        {
          const unsigned int primID = morton[start+i].index;
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          upper = max(upper,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }

        Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID));
        BBox3fa box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        if (N == 4)
          box_o.lower.a = unsigned(current.size());
#endif
        return NodeRecord(ref,box_o);
      }

    private:
      TriangleMesh* mesh;
      BVHBuilderMorton::BuildPrim* morton;
    };

    template<typename Mesh>
    struct CalculateMeshBounds
    {
      __forceinline CalculateMeshBounds (Mesh* mesh)
        : mesh(mesh) {}

      __forceinline const BBox3fa operator() (const BVHBuilderMorton::BuildPrim& morton) {
        return mesh->bounds(morton.index);
      }

    private:
      Mesh* mesh;
    };

    template<int N, typename Mesh, typename Primitive>
    class BVHNMeshBuilderMorton : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::AlignedNode AlignedNode;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecord NodeRecord;

    public:

      BVHNMeshBuilderMorton (BVH* bvh, Mesh* mesh, const size_t minLeafSize, const size_t maxLeafSize, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), mesh(mesh), morton(bvh->device,0), settings(N,BVH::maxBuildDepth,minLeafSize,maxLeafSize,singleThreadThreshold) {}

      /* build function */
      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
          morton.clear();
        }
        size_t numPrimitives = mesh->size();

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }

        /* preallocate arrays */
        morton.resize(numPrimitives);
        size_t bytesEstimated = numPrimitives*sizeof(AlignedNode)/(4*N) + size_t(1.2f*Primitive::blocks(numPrimitives)*sizeof(Primitive));
        size_t bytesMortonCodes = numPrimitives*sizeof(BVHBuilderMorton::BuildPrim);
        bytesEstimated = max(bytesEstimated,bytesMortonCodes); // the first allocation block is reused to sort the morton codes
        bvh->alloc.init(bytesMortonCodes,bytesMortonCodes,bytesEstimated);

        /* create morton code array */
        BVHBuilderMorton::BuildPrim* dest = (BVHBuilderMorton::BuildPrim*) bvh->alloc.specialAlloc(bytesMortonCodes);
        size_t numPrimitivesGen = createMortonCodeArray<Mesh>(mesh,morton,bvh->scene->progressInterface);

        /* create BVH */
        SetBVHNBounds<N> setBounds(bvh);
        CreateMortonLeaf<N,Primitive> createLeaf(mesh,morton.data());
        CalculateMeshBounds<Mesh> calculateBounds(mesh);
        auto root = BVHBuilderMorton::build<NodeRecord>(
          typename BVH::CreateAlloc(bvh),
          typename BVH::AlignedNode::Create(),
          setBounds,createLeaf,calculateBounds,bvh->scene->progressInterface,
          morton.data(),dest,numPrimitivesGen,settings);

        bvh->set(root.ref,LBBox3fa(root.bounds),numPrimitives);

#if ROTATE_TREE
        if (N == 4)
        {
          for (int i=0; i<ROTATE_TREE; i++)
            BVHNRotate<N>::rotate(bvh->root);
          bvh->clearBarrier(bvh->root);
        }
#endif

        /* clear temporary data for static geometry */
        if (bvh->scene->isStaticAccel())
        {
          morton.clear();
          bvh->shrink();
        }
        bvh->cleanup();
      }

      void clear() {
        morton.clear();
      }

    private:
      BVH* bvh;
      Mesh* mesh;
      mvector<BVHBuilderMorton::BuildPrim> morton;
      BVHBuilderMorton::Settings settings;
    };

#if defined(EMBREE_GEOMETRY_TRIANGLE)
    Builder* BVH4Triangle4MeshBuilderMortonGeneral  (void* bvh, TriangleMesh* mesh, size_t mode)
    {
      return new class BVHNMeshBuilderMorton<4,TriangleMesh,Triangle4> ((BVH4*)bvh,mesh,4,4);
    }
#if defined(__AVX__)
    Builder* BVH8Triangle4MeshBuilderMortonGeneral  (void* bvh, TriangleMesh* mesh, size_t mode)
    {
      return new class BVHNMeshBuilderMorton<8,TriangleMesh,Triangle4> ((BVH8*)bvh,mesh,4,4);
    }
#endif
#endif
  }
}
