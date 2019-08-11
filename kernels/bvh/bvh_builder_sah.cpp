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
#include "bvh_builder.h"
#include "../builders/primrefgen.h"
#include "../builders/splitter.h"

#include "../geometry/triangle.h"

#include "../common/state.h"
#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

#define PROFILE 0
#define PROFILE_RUNS 20

namespace embree
{
  namespace isa
  {
    template<int N, typename Primitive>
    struct CreateLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      __forceinline CreateLeaf (BVH* bvh) : bvh(bvh) {}

      __forceinline NodeRef operator() (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) const
      {
        size_t n = set.size();
        size_t items = Primitive::blocks(n);
        size_t start = set.begin();
        Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,set.end(),bvh->scene);
        }
        return node;
      }

      BVH* bvh;
    };


    template<int N, typename Primitive>
    struct CreateLeafQuantized
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      __forceinline CreateLeafQuantized (BVH* bvh) : bvh(bvh) {}

      __forceinline NodeRef operator() (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) const
      {
        size_t n = set.size();
        size_t items = Primitive::blocks(n);
        size_t start = set.begin();
        Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,set.end(),bvh->scene);
        }
        return node;
      }

      BVH* bvh;
    };

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      GeneralBVHBuilder::Settings settings;
      bool primrefarrayalloc;

      BVHNBuilderSAH (BVH* bvh,
                      Scene* scene,
                      const size_t sahBlockSize,
                      const float intCost,
                      const size_t minLeafSize,
                      const size_t maxLeafSize,
                      const size_t mode,
                      bool primrefarrayalloc = false)
        : bvh(bvh), scene(scene), mesh(nullptr),
          prims(scene->device,0),
          settings(sahBlockSize,
                   minLeafSize,
                   min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks),
                   travCost, intCost,
                   DEFAULT_SINGLE_THREAD_THRESHOLD),
          primrefarrayalloc(primrefarrayalloc)
      {
      }

      BVHNBuilderSAH (BVH* bvh,
                      Mesh* mesh,
                      const size_t sahBlockSize,
                      const float intCost,
                      const size_t minLeafSize,
                      const size_t maxLeafSize,
                      const size_t mode)
        : bvh(bvh),
          scene(nullptr),
          mesh(mesh),
          prims(bvh->device,0),
          settings(sahBlockSize,
                   minLeafSize,
                   min(maxLeafSize, Primitive::max_size()*BVH::maxLeafBlocks),
                   travCost, intCost,
                   DEFAULT_SINGLE_THREAD_THRESHOLD),
          primrefarrayalloc(false) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build()
      {
        printf("BVHNBuilderSAH::build\n");
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
        }

        /* if we use the primrefarray for allocations we have to take
        it back from the BVH */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.unshare(prims);

	/* skip build for empty scene */
        const size_t numPrimitives =
          mesh ? mesh->size() : (size_t)scene->world.numTriangles;

        //scene->getNumPrimitives<Mesh,false>();
        if (numPrimitives == 0) {
          bvh->clear();
          prims.clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

            /* create primref array */
            if (primrefarrayalloc) {
              settings.primrefarrayalloc = numPrimitives/1000;
              if (settings.primrefarrayalloc < 1000)
                settings.primrefarrayalloc = inf;
            }

            /* enable os_malloc for two level build */
            if (mesh)
              bvh->alloc.setOSallocation(true);

            /* initialize allocator */
            const size_t node_bytes = numPrimitives*sizeof(typename BVH::AlignedNodeMB)/(4*N);
            const size_t leaf_bytes =
              size_t(1.2 * Primitive::blocks(numPrimitives) * sizeof(TriangleM<4>));
            bvh->alloc.init_estimate(node_bytes+leaf_bytes);
            settings.singleThreadThreshold =
              bvh->alloc.fixSingleThreadThreshold(N,
                                                  DEFAULT_SINGLE_THREAD_THRESHOLD,
                                                  numPrimitives,
                                                  node_bytes+leaf_bytes);
            prims.resize(numPrimitives);

            PrimInfo pinfo = mesh ?
              createPrimRefArray(mesh,prims,bvh->scene->progressInterface) :
              createPrimRefArray(scene,Mesh::geom_type,false,prims,bvh->scene->progressInterface);

            /* pinfo might has zero size due to invalid geometry */
            if (unlikely(pinfo.size() == 0))
            {
              bvh->clear();
              prims.clear();
              return;
            }

            /* call BVH builder */
            NodeRef root = BVHNBuilderVirtual<N>::build(
              &bvh->alloc,
              CreateLeaf<N,TriangleM<4>>(bvh),
              bvh->scene->progressInterface,
              prims.data(),
              pinfo,
              settings);
            bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
            bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));

#if PROFILE
          });
#endif

        /* if we allocated using the primrefarray we have to keep it alive */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.share(prims);

        /* for static geometries we can do some cleanups */
        else if (scene && scene->isStaticAccel()) {
          bvh->shrink();
          prims.clear();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    Builder*
    BVH4Triangle4MeshBuilderSAH(void* bvh, TriangleMesh* mesh, size_t mode)
    {
      return new BVHNBuilderSAH<4,TriangleMesh,TriangleM<4>>(
        (BVH4*)bvh,
        mesh, 4, 1.0f, 4, inf, mode);
    }
    Builder*
    BVH4Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode)
    {
      return new BVHNBuilderSAH<4,TriangleMesh,TriangleM<4>>(
        (BVH4*)bvh,
        scene, 4, 1.0f, 4, inf, mode);
    }
#if defined(__AVX__)
    Builder*
    BVH8Triangle4MeshBuilderSAH(void* bvh, TriangleMesh* mesh, size_t mode)
    {
      return new BVHNBuilderSAH<8,TriangleMesh,TriangleM<4>>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode);
    }
    Builder*
    BVH8Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode)
    {
      return new BVHNBuilderSAH<8,TriangleMesh,TriangleM<4>>((BVH8*)bvh,scene,4,1.0f,4,inf,mode);
    }
#endif
  }
}
