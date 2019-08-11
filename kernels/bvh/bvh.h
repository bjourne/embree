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

#include "../common/default.h"
#include "../common/alloc.h"
#include "../common/accel.h"
#include "../common/device.h"
#include "../common/scene.h"
#include "../common/ray.h"

namespace embree
{
  /*! flags used to enable specific node types in intersectors */
  enum BVHNodeFlags
  {
    BVH_FLAG_ALIGNED_NODE = 0x00001,
    BVH_FLAG_ALIGNED_NODE_MB = 0x00010,
    BVH_FLAG_UNALIGNED_NODE = 0x00100,
    BVH_FLAG_UNALIGNED_NODE_MB = 0x01000,
    BVH_FLAG_QUANTIZED_NODE = 0x100000,
    BVH_FLAG_ALIGNED_NODE_MB4D = 0x1000000,

    /* short versions */
    BVH_AN1 = BVH_FLAG_ALIGNED_NODE,
    BVH_AN2 = BVH_FLAG_ALIGNED_NODE_MB,
    BVH_AN2_AN4D = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_ALIGNED_NODE_MB4D,
    BVH_UN1 = BVH_FLAG_UNALIGNED_NODE,
    BVH_UN2 = BVH_FLAG_UNALIGNED_NODE_MB,
    BVH_MB = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_UNALIGNED_NODE_MB | BVH_FLAG_ALIGNED_NODE_MB4D,
    BVH_AN1_UN1 = BVH_FLAG_ALIGNED_NODE | BVH_FLAG_UNALIGNED_NODE,
    BVH_AN2_UN2 = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_UNALIGNED_NODE_MB,
    BVH_AN2_AN4D_UN2 = BVH_FLAG_ALIGNED_NODE_MB | BVH_FLAG_ALIGNED_NODE_MB4D | BVH_FLAG_UNALIGNED_NODE_MB,
    BVH_QN1 = BVH_FLAG_QUANTIZED_NODE
  };

  /*! Multi BVH with N children. Each node stores the bounding box of
   * it's N children as well as N child references. */
  template<int N>
  class BVHN : public AccelData
  {
    ALIGNED_CLASS_(16);
  public:

    /*! forward declaration of node type */
    struct NodeRef;
    struct BaseNode;
    struct AlignedNode;
    struct UnalignedNode;
    struct QuantizedNode;

    /*! Number of bytes the nodes and primitives are minimally aligned to.*/
    static const size_t byteAlignment = 16;
    static const size_t byteNodeAlignment = 4*N;

    /*! highest address bit is used as barrier for some algorithms */
    static const size_t barrier_mask = (1LL << (8*sizeof(size_t)-1));

    /*! Masks the bits that store the number of items per leaf. */
    static const size_t align_mask = byteAlignment-1;
    static const size_t items_mask = byteAlignment - 1;

    /*! different supported node types */
    static const size_t tyAlignedNode = 0;
    static const size_t tyAlignedNodeMB = 1;
    static const size_t tyAlignedNodeMB4D = 6;
    static const size_t tyUnalignedNode = 2;
    static const size_t tyUnalignedNodeMB = 3;
    static const size_t tyQuantizedNode = 5;
    static const size_t tyLeaf = 8;

    /*! Empty node */
    static const size_t emptyNode = tyLeaf;

    /*! Invalid node, used as marker in traversal */
    static const size_t invalidNode = (((size_t)-1) & (~items_mask)) | (tyLeaf+0);
    static const size_t popRay      = (((size_t)-1) & (~items_mask)) | (tyLeaf+1);

    /*! Maximum depth of the BVH. */
    static const size_t maxBuildDepth = 32;
    static const size_t maxBuildDepthLeaf = maxBuildDepth+8;
    static const size_t maxDepth = 2*maxBuildDepthLeaf; // 2x because of two level builder

    /*! Maximum number of primitive blocks in a leaf. 15 - 8 = 7 */
    static const size_t maxLeafBlocks = items_mask - tyLeaf;

  public:

    /*! Builder interface to create allocator */
    struct CreateAlloc : public FastAllocator::Create {
      __forceinline CreateAlloc (BVHN* bvh) : FastAllocator::Create(&bvh->alloc) {}
    };

    /*! Pointer that points to a node or a list of primitives */
    struct NodeRef
    {
      /*! Default constructor */
      __forceinline NodeRef () {}

      /*! Construction from integer */
      __forceinline NodeRef (size_t ptr) : ptr(ptr) {}

      /*! Cast to size_t */
      __forceinline operator size_t() const { return ptr; }

      /*! Prefetches the node this reference points to */
      __forceinline void prefetch(int types=0) const {
#if  defined(__AVX512PF__) // MIC
          if (types != BVH_FLAG_QUANTIZED_NODE) {
            prefetchL2(((char*)ptr)+0*64);
            prefetchL2(((char*)ptr)+1*64);
            if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
              prefetchL2(((char*)ptr)+2*64);
              prefetchL2(((char*)ptr)+3*64);
            }
            if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
              /* KNL still needs L2 prefetches for large nodes */
              prefetchL2(((char*)ptr)+4*64);
              prefetchL2(((char*)ptr)+5*64);
              prefetchL2(((char*)ptr)+6*64);
              prefetchL2(((char*)ptr)+7*64);
            }
          }
          else
          {
            /* todo: reduce if 32bit offsets are enabled */
            prefetchL2(((char*)ptr)+0*64);
            prefetchL2(((char*)ptr)+1*64);
            prefetchL2(((char*)ptr)+2*64);
          }
#else
          if (types != BVH_FLAG_QUANTIZED_NODE) {
            prefetchL1(((char*)ptr)+0*64);
            prefetchL1(((char*)ptr)+1*64);
            if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
              prefetchL1(((char*)ptr)+2*64);
              prefetchL1(((char*)ptr)+3*64);
            }
            if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
              /* deactivate for large nodes on Xeon, as it introduces regressions */
              //prefetchL1(((char*)ptr)+4*64);
              //prefetchL1(((char*)ptr)+5*64);
              //prefetchL1(((char*)ptr)+6*64);
              //prefetchL1(((char*)ptr)+7*64);
            }
          }
          else
          {
            /* todo: reduce if 32bit offsets are enabled */
            prefetchL1(((char*)ptr)+0*64);
            prefetchL1(((char*)ptr)+1*64);
            prefetchL1(((char*)ptr)+2*64);
          }
#endif
      }

      __forceinline void prefetchLLC(int types=0) const {
        embree::prefetchL2(((char*)ptr)+0*64);
        embree::prefetchL2(((char*)ptr)+1*64);
        if (types != BVH_FLAG_QUANTIZED_NODE) {
          if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
            embree::prefetchL2(((char*)ptr)+2*64);
            embree::prefetchL2(((char*)ptr)+3*64);
          }
          if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
            embree::prefetchL2(((char*)ptr)+4*64);
            embree::prefetchL2(((char*)ptr)+5*64);
            embree::prefetchL2(((char*)ptr)+6*64);
            embree::prefetchL2(((char*)ptr)+7*64);
          }
        }
      }

      __forceinline void prefetch_L1(int types=0) const {
        embree::prefetchL1(((char*)ptr)+0*64);
        embree::prefetchL1(((char*)ptr)+1*64);
        if (types != BVH_FLAG_QUANTIZED_NODE) {
          if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
            embree::prefetchL1(((char*)ptr)+2*64);
            embree::prefetchL1(((char*)ptr)+3*64);
          }
          if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
            embree::prefetchL1(((char*)ptr)+4*64);
            embree::prefetchL1(((char*)ptr)+5*64);
            embree::prefetchL1(((char*)ptr)+6*64);
            embree::prefetchL1(((char*)ptr)+7*64);
          }
        }
      }

      __forceinline void prefetch_L2(int types=0) const {
        embree::prefetchL2(((char*)ptr)+0*64);
        embree::prefetchL2(((char*)ptr)+1*64);
        if (types != BVH_FLAG_QUANTIZED_NODE) {
          if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
            embree::prefetchL2(((char*)ptr)+2*64);
            embree::prefetchL2(((char*)ptr)+3*64);
          }
          if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
            embree::prefetchL2(((char*)ptr)+4*64);
            embree::prefetchL2(((char*)ptr)+5*64);
            embree::prefetchL2(((char*)ptr)+6*64);
            embree::prefetchL2(((char*)ptr)+7*64);
          }
        }
      }


      __forceinline void prefetchW(int types=0) const {
        embree::prefetchEX(((char*)ptr)+0*64);
        embree::prefetchEX(((char*)ptr)+1*64);
        if ((N >= 8) || (types > BVH_FLAG_ALIGNED_NODE)) {
          embree::prefetchEX(((char*)ptr)+2*64);
          embree::prefetchEX(((char*)ptr)+3*64);
        }
        if ((N >= 8) && (types > BVH_FLAG_ALIGNED_NODE)) {
          embree::prefetchEX(((char*)ptr)+4*64);
          embree::prefetchEX(((char*)ptr)+5*64);
          embree::prefetchEX(((char*)ptr)+6*64);
          embree::prefetchEX(((char*)ptr)+7*64);
        }
      }

      /*! Sets the barrier bit. */
      __forceinline void setBarrier() {
#if defined(__X86_64__)
        assert(!isBarrier());
        ptr |= barrier_mask;
#else
        assert(false);
#endif
      }

      /*! Clears the barrier bit. */
      __forceinline void clearBarrier() {
#if defined(__X86_64__)
        ptr &= ~barrier_mask;
#else
        assert(false);
#endif
      }

      /*! Checks if this is an barrier. A barrier tells the top level tree rotations how deep to enter the tree. */
      __forceinline bool isBarrier() const { return (ptr & barrier_mask) != 0; }

      /*! checks if this is a leaf */
      __forceinline size_t isLeaf() const { return ptr & tyLeaf; }

      /*! returns node type */
      __forceinline int type() const { return ptr & (size_t)align_mask; }

      /*! checks if this is a node */
      __forceinline int isAlignedNode() const { return (ptr & (size_t)align_mask) == tyAlignedNode; }

      /*! checks if this is a node with unaligned bounding boxes */
      __forceinline int isUnalignedNode() const { return (ptr & (size_t)align_mask) == tyUnalignedNode; }

      /*! checks if this is a quantized node */
      __forceinline int isQuantizedNode() const { return (ptr & (size_t)align_mask) == tyQuantizedNode; }

      /*! returns base node pointer */
      __forceinline BaseNode* baseNode(int types)
      {
        assert(!isLeaf());
        return (BaseNode*)(ptr & ~(size_t)align_mask);
      }
      __forceinline const BaseNode* baseNode(int types) const
      {
        assert(!isLeaf());
        return (const BaseNode*)(ptr & ~(size_t)align_mask);
      }

      /*! returns node pointer */
      __forceinline       AlignedNode* alignedNode()       { assert(isAlignedNode()); return (      AlignedNode*)ptr; }
      __forceinline const AlignedNode* alignedNode() const { assert(isAlignedNode()); return (const AlignedNode*)ptr; }

      /*! returns unaligned node pointer */
      __forceinline       UnalignedNode* unalignedNode()       { assert(isUnalignedNode()); return (      UnalignedNode*)(ptr & ~(size_t)align_mask); }
      __forceinline const UnalignedNode* unalignedNode() const { assert(isUnalignedNode()); return (const UnalignedNode*)(ptr & ~(size_t)align_mask); }

      /*! returns quantized node pointer */
    __forceinline       QuantizedNode* quantizedNode()
    {
      assert(isQuantizedNode());
      return (      QuantizedNode*)(ptr  & ~(size_t)align_mask );
    }
    __forceinline const QuantizedNode* quantizedNode() const {
      assert(isQuantizedNode());
      return (const QuantizedNode*)(ptr  & ~(size_t)align_mask );
    }

    /*! returns leaf pointer */
    __forceinline char* leaf(size_t& num) const
    {
      assert(isLeaf());
      num = (ptr & (size_t)items_mask)-tyLeaf;
      return (char*)(ptr & ~(size_t)align_mask);
    }

      /*! clear all bit flags */
      __forceinline void clearFlags() {
        ptr &= ~(size_t)align_mask;
      }

      /*! returns the wideness */
      __forceinline size_t getN() const { return N; }

    private:
      size_t ptr;
    };

  /*! BVHN Base Node */
    struct BaseNode
    {
      /*! Clears the node. */
      __forceinline void clear() {
        for (size_t i=0; i<N; i++) children[i] = emptyNode;
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

      /*! verifies the node */
      __forceinline bool verify() const
      {
        for (size_t i=0; i<N; i++) {
          if (child(i) == BVHN::emptyNode) {
            for (; i<N; i++) {
              if (child(i) != BVHN::emptyNode)
                return false;
            }
            break;
          }
        }
        return true;
      }

      NodeRef children[N];    //!< Pointer to the N children (can be a node or leaf)
    };

    /*! BVHN AlignedNode */
    struct AlignedNode : public BaseNode
    {
      using BaseNode::children;

      struct Create
      {
        __forceinline NodeRef operator() (const FastAllocator::CachedAllocator& alloc, size_t numChildren = 0) const
        {
          AlignedNode* node = (AlignedNode*) alloc.malloc0(sizeof(AlignedNode),byteNodeAlignment); node->clear();
          return BVHN::encodeNode(node);
        }
      };

      struct Set
      {
        __forceinline void operator() (NodeRef node, size_t i, NodeRef child, const BBox3fa& bounds) const {
          node.alignedNode()->setRef(i,child);
          node.alignedNode()->setBounds(i,bounds);
        }
      };

      struct Create2
      {
        template<typename BuildRecord>
        __forceinline NodeRef operator() (BuildRecord* children, const size_t num, const FastAllocator::CachedAllocator& alloc) const
        {
          AlignedNode* node = (AlignedNode*) alloc.malloc0(sizeof(AlignedNode), byteNodeAlignment); node->clear();
          for (size_t i=0; i<num; i++) node->setBounds(i,children[i].bounds());
          return encodeNode(node);
        }
      };

      struct Set2
      {
        template<typename BuildRecord>
        __forceinline NodeRef operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRef ref, NodeRef* children, const size_t num) const
        {
          AlignedNode* node = ref.alignedNode();
          for (size_t i=0; i<num; i++) node->setRef(i,children[i]);
          return ref;
        }
      };

      struct Set3
      {
        Set3 (FastAllocator* allocator, PrimRef* prims)
        : allocator(allocator), prims(prims) {}

        template<typename BuildRecord>
        __forceinline NodeRef operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRef ref, NodeRef* children, const size_t num) const
        {
          AlignedNode* node = ref.alignedNode();
          for (size_t i=0; i<num; i++) node->setRef(i,children[i]);

          if (unlikely(precord.alloc_barrier))
          {
            PrimRef* begin = &prims[precord.prims.begin()];
            PrimRef* end   = &prims[precord.prims.end()]; // FIXME: extended end for spatial split builder!!!!!
            size_t bytes = (size_t)end - (size_t)begin;
            allocator->addBlock(begin,bytes);
          }

          return ref;
        }

        FastAllocator* const allocator;
        PrimRef* const prims;
      };

      /*! Clears the node. */
      __forceinline void clear() {
        lower_x = lower_y = lower_z = pos_inf;
        upper_x = upper_y = upper_z = neg_inf;
        BaseNode::clear();
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void setRef(size_t i, const NodeRef& ref) {
        assert(i < N);
        children[i] = ref;
      }

      /*! Sets bounding box of child. */
      __forceinline void setBounds(size_t i, const BBox3fa& bounds)
      {
        assert(i < N);
        lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
        upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
      }

      /*! Sets bounding box and ID of child. */
      __forceinline void set(size_t i, const NodeRef& ref, const BBox3fa& bounds) {
        setBounds(i,bounds);
        children[i] = ref;
      }

      /*! Returns bounds of node. */
      __forceinline BBox3fa bounds() const {
        const Vec3fa lower(reduce_min(lower_x),reduce_min(lower_y),reduce_min(lower_z));
        const Vec3fa upper(reduce_max(upper_x),reduce_max(upper_y),reduce_max(upper_z));
        return BBox3fa(lower,upper);
      }

      /*! Returns bounds of specified child. */
      __forceinline BBox3fa bounds(size_t i) const
      {
        assert(i < N);
        const Vec3fa lower(lower_x[i],lower_y[i],lower_z[i]);
        const Vec3fa upper(upper_x[i],upper_y[i],upper_z[i]);
        return BBox3fa(lower,upper);
      }

      /*! Returns extent of bounds of specified child. */
      __forceinline Vec3fa extend(size_t i) const {
        return bounds(i).size();
      }

      /*! Returns bounds of all children (implemented later as specializations) */
      __forceinline void bounds(BBox<vfloat4>& bounds0, BBox<vfloat4>& bounds1, BBox<vfloat4>& bounds2, BBox<vfloat4>& bounds3) const {} // N = 4

      /*! swap two children of the node */
      __forceinline void swap(size_t i, size_t j)
      {
        assert(i<N && j<N);
        std::swap(children[i],children[j]);
        std::swap(lower_x[i],lower_x[j]);
        std::swap(lower_y[i],lower_y[j]);
        std::swap(lower_z[i],lower_z[j]);
        std::swap(upper_x[i],upper_x[j]);
        std::swap(upper_y[i],upper_y[j]);
        std::swap(upper_z[i],upper_z[j]);
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

      /*! output operator */
      friend std::ostream& operator<<(std::ostream& o, const AlignedNode& n)
      {
        o << "AlignedNode { " << std::endl;
        o << "  lower_x " << n.lower_x << std::endl;
        o << "  upper_x " << n.upper_x << std::endl;
        o << "  lower_y " << n.lower_y << std::endl;
        o << "  upper_y " << n.upper_y << std::endl;
        o << "  lower_z " << n.lower_z << std::endl;
        o << "  upper_z " << n.upper_z << std::endl;
        o << "  children = ";
        for (size_t i=0; i<N; i++) o << n.children[i] << " ";
        o << std::endl;
        o << "}" << std::endl;
        return o;
      }

    public:
      vfloat<N> lower_x;           //!< X dimension of lower bounds of all N children.
      vfloat<N> upper_x;           //!< X dimension of upper bounds of all N children.
      vfloat<N> lower_y;           //!< Y dimension of lower bounds of all N children.
      vfloat<N> upper_y;           //!< Y dimension of upper bounds of all N children.
      vfloat<N> lower_z;           //!< Z dimension of lower bounds of all N children.
      vfloat<N> upper_z;           //!< Z dimension of upper bounds of all N children.
    };

    /*! Node with unaligned bounds */
    struct UnalignedNode : public BaseNode
    {
      using BaseNode::children;

      struct Create
      {
        __forceinline NodeRef operator() (const FastAllocator::CachedAllocator& alloc) const
        {
          UnalignedNode* node = (UnalignedNode*) alloc.malloc0(sizeof(UnalignedNode),byteNodeAlignment); node->clear();
          return BVHN::encodeNode(node);
        }
      };

      struct Set
      {
        __forceinline void operator() (NodeRef node, size_t i, NodeRef child, const OBBox3fa& bounds) const {
          node.unalignedNode()->setRef(i,child);
          node.unalignedNode()->setBounds(i,bounds);
        }
      };

      /*! Clears the node. */
      __forceinline void clear()
      {
        naabb.l.vx = Vec3fa(nan);
        naabb.l.vy = Vec3fa(nan);
        naabb.l.vz = Vec3fa(nan);
        naabb.p    = Vec3fa(nan);
        BaseNode::clear();
      }

      /*! Sets bounding box. */
      __forceinline void setBounds(size_t i, const OBBox3fa& b)
      {
        assert(i < N);

        AffineSpace3fa space = b.space;
        space.p -= b.bounds.lower;
        space = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19f),b.bounds.upper-b.bounds.lower))*space;

        naabb.l.vx.x[i] = space.l.vx.x;
        naabb.l.vx.y[i] = space.l.vx.y;
        naabb.l.vx.z[i] = space.l.vx.z;

        naabb.l.vy.x[i] = space.l.vy.x;
        naabb.l.vy.y[i] = space.l.vy.y;
        naabb.l.vy.z[i] = space.l.vy.z;

        naabb.l.vz.x[i] = space.l.vz.x;
        naabb.l.vz.y[i] = space.l.vz.y;
        naabb.l.vz.z[i] = space.l.vz.z;

        naabb.p.x[i] = space.p.x;
        naabb.p.y[i] = space.p.y;
        naabb.p.z[i] = space.p.z;
      }

      /*! Sets ID of child. */
      __forceinline void setRef(size_t i, const NodeRef& ref) {
        assert(i < N);
        children[i] = ref;
      }

      /*! Returns the extent of the bounds of the ith child */
      __forceinline Vec3fa extent(size_t i) const {
        assert(i<N);
        const Vec3fa vx(naabb.l.vx.x[i],naabb.l.vx.y[i],naabb.l.vx.z[i]);
        const Vec3fa vy(naabb.l.vy.x[i],naabb.l.vy.y[i],naabb.l.vy.z[i]);
        const Vec3fa vz(naabb.l.vz.x[i],naabb.l.vz.y[i],naabb.l.vz.z[i]);
        return rsqrt(vx*vx + vy*vy + vz*vz);
      }

      /*! Returns reference to specified child */
      __forceinline       NodeRef& child(size_t i)       { assert(i<N); return children[i]; }
      __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

      /*! output operator */
      friend std::ostream& operator<<(std::ostream& o, const UnalignedNode& n)
      {
        o << "UnAlignedNode { " << n.naabb << " } " << std::endl;
        return o;
      }

    public:
      AffineSpace3vf<N> naabb;   //!< non-axis aligned bounding boxes (bounds are [0,1] in specified space)
    };

    /*! BVHN Quantized Node */
    struct __aligned(8) QuantizedBaseNode
    {
      typedef unsigned char T;
      static const T MIN_QUAN = 0;
      static const T MAX_QUAN = 255;

      /*! Clears the node. */
      __forceinline void clear() {
        for (size_t i=0; i<N; i++) lower_x[i] = lower_y[i] = lower_z[i] = MAX_QUAN;
        for (size_t i=0; i<N; i++) upper_x[i] = upper_y[i] = upper_z[i] = MIN_QUAN;
      }

      /*! Returns bounds of specified child. */
      __forceinline BBox3fa bounds(size_t i) const
      {
        assert(i < N);
        const Vec3fa lower(madd(scale.x,(float)lower_x[i],start.x),
                           madd(scale.y,(float)lower_y[i],start.y),
                           madd(scale.z,(float)lower_z[i],start.z));
        const Vec3fa upper(madd(scale.x,(float)upper_x[i],start.x),
                           madd(scale.y,(float)upper_y[i],start.y),
                           madd(scale.z,(float)upper_z[i],start.z));
        return BBox3fa(lower,upper);
      }

      /*! Returns extent of bounds of specified child. */
      __forceinline Vec3fa extent(size_t i) const {
        return bounds(i).size();
      }

      static __forceinline void init_dim(const vfloat<N> &lower,
                                         const vfloat<N> &upper,
                                         T lower_quant[N],
                                         T upper_quant[N],
                                         float &start,
                                         float &scale)
      {
        /* quantize bounds */
        const vbool<N> m_valid = lower != vfloat<N>(pos_inf);
        const float minF = reduce_min(lower);
        const float maxF = reduce_max(upper);
        float diff = (1.0f+2.0f*float(ulp))*(maxF - minF);
        float decode_scale = diff / float(MAX_QUAN);
        if (decode_scale == 0.0f) decode_scale = 2.0f*FLT_MIN; // result may have been flushed to zero
        assert(madd(decode_scale,float(MAX_QUAN),minF) >= maxF);
        const float encode_scale = float(MAX_QUAN) / diff;
        vint<N> ilower = max(vint<N>(floor((lower - vfloat<N>(minF))*vfloat<N>(encode_scale))),MIN_QUAN);
        vint<N> iupper = min(vint<N>(ceil ((upper - vfloat<N>(minF))*vfloat<N>(encode_scale))),MAX_QUAN);

        /* lower/upper correction */
        vbool<N> m_lower_correction = (madd(vfloat<N>(ilower),decode_scale,minF)) > lower;
        vbool<N> m_upper_correction = (madd(vfloat<N>(iupper),decode_scale,minF)) < upper;
        ilower = max(select(m_lower_correction,ilower-1,ilower),MIN_QUAN);
        iupper = min(select(m_upper_correction,iupper+1,iupper),MAX_QUAN);

        /* disable invalid lanes */
        ilower = select(m_valid,ilower,MAX_QUAN);
        iupper = select(m_valid,iupper,MIN_QUAN);

        /* store as uchar to memory */
        vint<N>::store(lower_quant,ilower);
        vint<N>::store(upper_quant,iupper);
        start = minF;
        scale = decode_scale;

#if defined(DEBUG)
        vfloat<N> extract_lower( vint<N>::loadu(lower_quant) );
        vfloat<N> extract_upper( vint<N>::loadu(upper_quant) );
        vfloat<N> final_extract_lower = madd(extract_lower,decode_scale,minF);
        vfloat<N> final_extract_upper = madd(extract_upper,decode_scale,minF);
        assert( (movemask(final_extract_lower <= lower ) & movemask(m_valid)) == movemask(m_valid));
        assert( (movemask(final_extract_upper >= upper ) & movemask(m_valid)) == movemask(m_valid));
#endif
      }

      __forceinline void init_dim(AlignedNode& node)
      {
        init_dim(node.lower_x,node.upper_x,lower_x,upper_x,start.x,scale.x);
        init_dim(node.lower_y,node.upper_y,lower_y,upper_y,start.y,scale.y);
        init_dim(node.lower_z,node.upper_z,lower_z,upper_z,start.z,scale.z);
      }

      __forceinline vbool<N> validMask() const { return vint<N>::loadu(lower_x) <= vint<N>::loadu(upper_x); }

#if defined(__AVX512F__) // KNL
      __forceinline vbool16 validMask16() const { return le(0xff,vint<16>::loadu(lower_x),vint<16>::loadu(upper_x)); }
#endif
      __forceinline vfloat<N> dequantizeLowerX() const { return madd(vfloat<N>(vint<N>::loadu(lower_x)),scale.x,vfloat<N>(start.x)); }

      __forceinline vfloat<N> dequantizeUpperX() const { return madd(vfloat<N>(vint<N>::loadu(upper_x)),scale.x,vfloat<N>(start.x)); }

      __forceinline vfloat<N> dequantizeLowerY() const { return madd(vfloat<N>(vint<N>::loadu(lower_y)),scale.y,vfloat<N>(start.y)); }

      __forceinline vfloat<N> dequantizeUpperY() const { return madd(vfloat<N>(vint<N>::loadu(upper_y)),scale.y,vfloat<N>(start.y)); }

      __forceinline vfloat<N> dequantizeLowerZ() const { return madd(vfloat<N>(vint<N>::loadu(lower_z)),scale.z,vfloat<N>(start.z)); }

      __forceinline vfloat<N> dequantizeUpperZ() const { return madd(vfloat<N>(vint<N>::loadu(upper_z)),scale.z,vfloat<N>(start.z)); }

      template <int M>
      __forceinline vfloat<M> dequantize(const size_t offset) const { return vfloat<M>(vint<M>::loadu(all_planes+offset)); }

#if defined(__AVX512F__)
      __forceinline vfloat16 dequantizeLowerUpperX(const vint16 &p) const { return madd(vfloat16(permute(vint<16>::loadu(lower_x),p)),scale.x,vfloat16(start.x)); }
      __forceinline vfloat16 dequantizeLowerUpperY(const vint16 &p) const { return madd(vfloat16(permute(vint<16>::loadu(lower_y),p)),scale.y,vfloat16(start.y)); }
      __forceinline vfloat16 dequantizeLowerUpperZ(const vint16 &p) const { return madd(vfloat16(permute(vint<16>::loadu(lower_z),p)),scale.z,vfloat16(start.z)); }
#endif

      union {
        struct {
          T lower_x[N]; //!< 8bit discretized X dimension of lower bounds of all N children
          T upper_x[N]; //!< 8bit discretized X dimension of upper bounds of all N children
          T lower_y[N]; //!< 8bit discretized Y dimension of lower bounds of all N children
          T upper_y[N]; //!< 8bit discretized Y dimension of upper bounds of all N children
          T lower_z[N]; //!< 8bit discretized Z dimension of lower bounds of all N children
          T upper_z[N]; //!< 8bit discretized Z dimension of upper bounds of all N children
        };
        T all_planes[6*N];
      };

      Vec3f start;
      Vec3f scale;

      friend std::ostream& operator<<(std::ostream& o, const QuantizedBaseNode& n)
      {
        o << "QuantizedBaseNode { " << std::endl;
        o << "  start   " << n.start << std::endl;
        o << "  scale   " << n.scale << std::endl;
        o << "  lower_x " << vuint<N>::loadu(n.lower_x) << std::endl;
        o << "  upper_x " << vuint<N>::loadu(n.upper_x) << std::endl;
        o << "  lower_y " << vuint<N>::loadu(n.lower_y) << std::endl;
        o << "  upper_y " << vuint<N>::loadu(n.upper_y) << std::endl;
        o << "  lower_z " << vuint<N>::loadu(n.lower_z) << std::endl;
        o << "  upper_z " << vuint<N>::loadu(n.upper_z) << std::endl;
        o << "}" << std::endl;
        return o;
      }

    };

    struct __aligned(8) QuantizedNode : public BaseNode, QuantizedBaseNode
    {
      using BaseNode::children;
      using QuantizedBaseNode::lower_x;
      using QuantizedBaseNode::upper_x;
      using QuantizedBaseNode::lower_y;
      using QuantizedBaseNode::upper_y;
      using QuantizedBaseNode::lower_z;
      using QuantizedBaseNode::upper_z;
      using QuantizedBaseNode::start;
      using QuantizedBaseNode::scale;
      using QuantizedBaseNode::init_dim;

      __forceinline void setRef(size_t i, const NodeRef& ref) {
        assert(i < N);
        children[i] = ref;
      }

      struct Create2
      {
        template<typename BuildRecord>
        __forceinline NodeRef operator() (BuildRecord* children, const size_t n, const FastAllocator::CachedAllocator& alloc) const
        {
          __aligned(64) AlignedNode node;
          node.clear();
          for (size_t i=0; i<n; i++) {
            node.setBounds(i,children[i].bounds());
          }
          QuantizedNode *qnode = (QuantizedNode*) alloc.malloc0(sizeof(QuantizedNode), byteAlignment);
          qnode->init(node);

          return (size_t)qnode | tyQuantizedNode;
        }
      };

      // struct Set2
      // {
      //   template<typename BuildRecord>
      //   __forceinline NodeRef operator() (const BuildRecord& precord, const BuildRecord* crecords, NodeRef ref, NodeRef* children, const size_t num) const
      //   {
      //     QuantizedNode* node = ref.quantizedNode();
      //     for (size_t i=0; i<num; i++) node->setRef(i,children[i]);
      //     return ref;
      //   }
      // };

      __forceinline void init(AlignedNode& node)
      {
        for (size_t i=0;i<N;i++) children[i] = emptyNode;
        init_dim(node);
      }

    };

    /*! swap the children of two nodes */
    __forceinline static void swap(AlignedNode* a, size_t i, AlignedNode* b, size_t j)
    {
      assert(i<N && j<N);
      std::swap(a->children[i],b->children[j]);
      std::swap(a->lower_x[i],b->lower_x[j]);
      std::swap(a->lower_y[i],b->lower_y[j]);
      std::swap(a->lower_z[i],b->lower_z[j]);
      std::swap(a->upper_x[i],b->upper_x[j]);
      std::swap(a->upper_y[i],b->upper_y[j]);
      std::swap(a->upper_z[i],b->upper_z[j]);
    }

    /*! compacts a node (moves empty children to the end) */
    __forceinline static void compact(AlignedNode* a)
    {
      /* find right most filled node */
      ssize_t j=N;
      for (j=j-1; j>=0; j--)
        if (a->child(j) != emptyNode)
          break;

      /* replace empty nodes with filled nodes */
      for (ssize_t i=0; i<j; i++) {
        if (a->child(i) == emptyNode) {
          a->swap(i,j);
          for (j=j-1; j>i; j--)
            if (a->child(j) != emptyNode)
              break;
        }
      }
    }

  public:

    /*! BVHN default constructor. */
    BVHN (Scene* scene);

    /*! BVHN destruction */
    ~BVHN ();

    /*! clears the acceleration structure */
    void clear();

    /*! sets BVH members after build */
    void set (NodeRef root, const LBBox3fa& bounds, size_t numPrimitives);

    /*! Clears the barrier bits of a subtree. */
    void clearBarrier(NodeRef& node);

    /*! lays out num large nodes of the BVH */
    void layoutLargeNodes(size_t num);
    NodeRef
    layoutLargeNodesRecursion(NodeRef& node,
                              const FastAllocator::CachedAllocator& allocator);

    /*! called by all builders before build starts */
    double preBuild(const std::string& builderName);

    /*! called by all builders after build ended */
    void postBuild(double t0);

    /*! allocator class */
    struct Allocator {
      BVHN* bvh;
      Allocator (BVHN* bvh) : bvh(bvh) {}
      __forceinline void* operator() (size_t bytes) const {
        return bvh->alloc._threadLocal()->malloc(&bvh->alloc,bytes);
      }
    };

    /*! shrink allocated memory */
    void shrink() { // FIXME: remove
    }

    /*! post build cleanup */
    void cleanup() {
      alloc.cleanup();
    }

  public:

    /*! Encodes a node */
    static __forceinline NodeRef encodeNode(AlignedNode* node) {
      assert(!((size_t)node & align_mask));
      return NodeRef((size_t) node);
    }

    /*! Encodes an unaligned node */
    static __forceinline NodeRef encodeNode(UnalignedNode* node) {
      return NodeRef((size_t) node | tyUnalignedNode);
    }

    /*! Encodes a leaf */
    static __forceinline NodeRef encodeLeaf(void* tri, size_t num) {
      assert(!((size_t)tri & align_mask));
      assert(num <= maxLeafBlocks);
      return NodeRef((size_t)tri | (tyLeaf+min(num,(size_t)maxLeafBlocks)));
    }

    /*! Encodes a leaf */
    static __forceinline NodeRef encodeTypedLeaf(void* ptr, size_t ty) {
      assert(!((size_t)ptr & align_mask));
      return NodeRef((size_t)ptr | (tyLeaf+ty));
    }

    /*! bvh data */
  public:
    Device* device;                    //!< device pointer
    Scene* scene;                      //!< scene pointer
    NodeRef root;                      //!< root node
    FastAllocator alloc;               //!< allocator used to allocate nodes

    /*! statistics data */
  public:
    size_t numPrimitives;              //!< number of primitives the BVH is build over
    size_t numVertices;                //!< number of vertices the BVH references

    /*! data arrays for special builders */
  public:
    std::vector<BVHN*> objects;
  };

  template<>
  __forceinline void BVHN<4>::AlignedNode::bounds(BBox<vfloat4>& bounds0,
                                                  BBox<vfloat4>& bounds1,
                                                  BBox<vfloat4>& bounds2,
                                                  BBox<vfloat4>& bounds3) const {
    transpose(lower_x, lower_y, lower_z, vfloat4(zero),
              bounds0.lower,bounds1.lower,bounds2.lower,bounds3.lower);
    transpose(upper_x,upper_y,upper_z,vfloat4(zero),
              bounds0.upper,bounds1.upper,bounds2.upper,bounds3.upper);
  }
  typedef BVHN<4> BVH4;
  typedef BVHN<8> BVH8;
}
