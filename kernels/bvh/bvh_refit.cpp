#include "bvh_refit.h"
#include "bvh_statistics.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/object.h"

namespace embree
{
namespace isa
{
static const size_t SINGLE_THREAD_THRESHOLD = 4*1024;

template<int N>
__forceinline bool compare(const typename BVHN<N>::NodeRef* a, const typename BVHN<N>::NodeRef* b)
{
    size_t sa = *(size_t*)&a->node()->lower_x;
    size_t sb = *(size_t*)&b->node()->lower_x;
    return sa < sb;
}

    template<int N>
    BVHNRefitter<N>::BVHNRefitter (BVH* bvh, const LeafBoundsInterface& leafBounds)
      : bvh(bvh), leafBounds(leafBounds), numSubTrees(0)
    {
    }

    template<int N>
    void BVHNRefitter<N>::refit()
    {
      if (bvh->numPrimitives <= SINGLE_THREAD_THRESHOLD) {
        bvh->bounds = LBBox3fa(recurse_bottom(bvh->root));
      }
      else
      {
        BBox3fa subTreeBounds[MAX_NUM_SUB_TREES];
        numSubTrees = 0;
        gather_subtree_refs(bvh->root,numSubTrees,0);
        if (numSubTrees)
          parallel_for(size_t(0), numSubTrees, size_t(1), [&](const range<size_t>& r) {
              for (size_t i=r.begin(); i<r.end(); i++) {
                NodeRef& ref = subTrees[i];
                subTreeBounds[i] = recurse_bottom(ref);
              }
            });

        numSubTrees = 0;
        bvh->bounds = LBBox3fa(refit_toplevel(bvh->root,numSubTrees,subTreeBounds,0));
      }
  }

    template<int N>
    void BVHNRefitter<N>::gather_subtree_refs(NodeRef& ref,
                                              size_t &subtrees,
                                              const size_t depth)
    {
      if (depth >= MAX_SUB_TREE_EXTRACTION_DEPTH)
      {
        assert(subtrees < MAX_NUM_SUB_TREES);
        subTrees[subtrees++] = ref;
        return;
      }

      if (ref.isAlignedNode())
      {
        AlignedNode* node = ref.alignedNode();
        for (size_t i=0; i<N; i++) {
          NodeRef& child = node->child(i);
          if (unlikely(child == BVH::emptyNode)) continue;
          gather_subtree_refs(child,subtrees,depth+1);
        }
      }
    }

    template<int N>
    BBox3fa BVHNRefitter<N>::refit_toplevel(NodeRef& ref,
                                            size_t &subtrees,
											const BBox3fa *const subTreeBounds,
                                            const size_t depth)
    {
      if (depth >= MAX_SUB_TREE_EXTRACTION_DEPTH)
      {
        assert(subtrees < MAX_NUM_SUB_TREES);
        assert(subTrees[subtrees] == ref);
        return subTreeBounds[subtrees++];
      }

      if (ref.isAlignedNode())
      {
        AlignedNode* node = ref.alignedNode();
        BBox3fa bounds[N];

        for (size_t i=0; i<N; i++)
        {
          NodeRef& child = node->child(i);

          if (unlikely(child == BVH::emptyNode))
            bounds[i] = BBox3fa(empty);
          else
            bounds[i] = refit_toplevel(child,subtrees,subTreeBounds,depth+1);
        }

        BBox3vf<N> boundsT = transpose<N>(bounds);

        /* set new bounds */
        node->lower_x = boundsT.lower.x;
        node->lower_y = boundsT.lower.y;
        node->lower_z = boundsT.lower.z;
        node->upper_x = boundsT.upper.x;
        node->upper_y = boundsT.upper.y;
        node->upper_z = boundsT.upper.z;

        return merge<N>(bounds);
      }
      else
        return leafBounds.leafBounds(ref);
    }

    // =========================================================
    // =========================================================
    // =========================================================


    template<int N>
    BBox3fa BVHNRefitter<N>::recurse_bottom(NodeRef& ref)
    {
      /* this is a leaf node */
      if (unlikely(ref.isLeaf()))
        return leafBounds.leafBounds(ref);

      /* recurse if this is an internal node */
      AlignedNode* node = ref.alignedNode();

      /* enable exclusive prefetch for >= AVX platforms */
#if defined(__AVX__)
      ref.prefetchW();
#endif
      BBox3fa bounds[N];

      for (size_t i=0; i<N; i++)
        if (unlikely(node->child(i) == BVH::emptyNode))
        {
          bounds[i] = BBox3fa(empty);
        }
      else
        bounds[i] = recurse_bottom(node->child(i));

      /* AOS to SOA transform */
      BBox3vf<N> boundsT = transpose<N>(bounds);

      /* set new bounds */
      node->lower_x = boundsT.lower.x;
      node->lower_y = boundsT.lower.y;
      node->lower_z = boundsT.lower.z;
      node->upper_x = boundsT.upper.x;
      node->upper_y = boundsT.upper.y;
      node->upper_z = boundsT.upper.z;

      return merge<N>(bounds);
    }

    template<int N, typename Mesh, typename Primitive>
    BVHNRefitT<N,Mesh,Primitive>::BVHNRefitT (BVH* bvh, Builder* builder, Mesh* mesh, size_t mode)
      : bvh(bvh), builder(builder), refitter(new BVHNRefitter<N>(bvh,*(typename BVHNRefitter<N>::LeafBoundsInterface*)this)), mesh(mesh) {}

    template<int N, typename Mesh, typename Primitive>
    void BVHNRefitT<N,Mesh,Primitive>::clear()
    {
      if (builder)
        builder->clear();
    }

    template<int N, typename Mesh, typename Primitive>
    void BVHNRefitT<N,Mesh,Primitive>::build()
    {
      if (mesh->topologyChanged()) {
        builder->build();
      }
      else
        refitter->refit();
    }

    template class BVHNRefitter<4>;
#if defined(__AVX__)
    template class BVHNRefitter<8>;
#endif

#if defined(EMBREE_GEOMETRY_TRIANGLE)
    Builder* BVH4Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode);
    Builder* BVH4Triangle4MeshRefitSAH  (void* accel, TriangleMesh* mesh, size_t mode) { return new BVHNRefitT<4,TriangleMesh,Triangle4> ((BVH4*)accel,BVH4Triangle4MeshBuilderSAH (accel,mesh,mode),mesh,mode); }
#if  defined(__AVX__)
    Builder* BVH8Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode);

    Builder* BVH8Triangle4MeshRefitSAH  (void* accel, TriangleMesh* mesh, size_t mode) { return new BVHNRefitT<8,TriangleMesh,Triangle4> ((BVH8*)accel,BVH8Triangle4MeshBuilderSAH (accel,mesh,mode),mesh,mode); }
#endif
#endif


  }
}
