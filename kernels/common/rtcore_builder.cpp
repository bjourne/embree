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

#define RTC_EXPORT_API

#include "default.h"
#include "device.h"
#include "scene.h"
#include "context.h"
#include "alloc.h"

#include "../builders/bvh_builder_sah.h"

namespace embree
{
  namespace isa // FIXME: support more ISAs for builders
  {
    struct BVH : public RefCount
    {
      BVH (Device* device)
        : device(device), allocator(device,true)
      {
        device->refInc();
      }

      ~BVH() {
        device->refDec();
      }

    public:
      Device* device;
      FastAllocator allocator;
    };

    // void* rtcBuildBVHBinnedSAH(const RTCBuildArguments* arguments)
    // {
    //   BVH* bvh = (BVH*) arguments->bvh;
    //   RTCBuildPrimitive* prims =  arguments->primitives;
    //   size_t primitiveCount = arguments->primitiveCount;
    //   RTCCreateNodeFunction createNode = arguments->createNode;
    //   RTCSetNodeChildrenFunction setNodeChildren = arguments->setNodeChildren;
    //   RTCSetNodeBoundsFunction setNodeBounds = arguments->setNodeBounds;
    //   RTCCreateLeafFunction createLeaf = arguments->createLeaf;
    //   RTCProgressMonitorFunction buildProgress = arguments->buildProgress;
    //   void* userPtr = arguments->userPtr;

    //   std::atomic<size_t> progress(0);

    //   /* calculate priminfo */
    //   auto computeBounds = [&](const range<size_t>& r) -> CentGeomBBox3fa
    //     {
    //       CentGeomBBox3fa bounds(empty);
    //       for (size_t j=r.begin(); j<r.end(); j++)
    //         bounds.extend((BBox3fa&)prims[j]);
    //       return bounds;
    //     };
    //   const CentGeomBBox3fa bounds =
    //     parallel_reduce(size_t(0),primitiveCount,size_t(1024),size_t(1024),CentGeomBBox3fa(empty), computeBounds, CentGeomBBox3fa::merge2);

    //   const PrimInfo pinfo(0,primitiveCount,bounds);

    //   /* build BVH */
    //   void* root = BVHBuilderBinnedSAH::build<void*>(

    //     /* thread local allocator for fast allocations */
    //     [&] () -> FastAllocator::CachedAllocator {
    //       return bvh->allocator.getCachedAllocator();
    //     },

    //     /* lambda function that creates BVH nodes */
    //     [&](BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, const FastAllocator::CachedAllocator& alloc) -> void*
    //     {
    //       void* node = createNode((RTCThreadLocalAllocator)&alloc, (unsigned int)N,userPtr);
    //       const RTCBounds* cbounds[GeneralBVHBuilder::MAX_BRANCHING_FACTOR];
    //       for (size_t i=0; i<N; i++) cbounds[i] = (const RTCBounds*) &children[i].prims.geomBounds;
    //       setNodeBounds(node,cbounds, (unsigned int)N,userPtr);
    //       return node;
    //     },

    //     /* lambda function that updates BVH nodes */
    //     [&](const BVHBuilderBinnedSAH::BuildRecord& precord, const BVHBuilderBinnedSAH::BuildRecord* crecords, void* node, void** children, const size_t N) -> void* {
    //       setNodeChildren(node,children, (unsigned int)N,userPtr);
    //       return node;
    //     },

    //     /* lambda function that creates BVH leaves */
    //     [&](const PrimRef* prims, const range<size_t>& range, const FastAllocator::CachedAllocator& alloc) -> void* {
    //       return createLeaf((RTCThreadLocalAllocator)&alloc,(RTCBuildPrimitive*)(prims+range.begin()),range.size(),userPtr);
    //     },

    //     /* progress monitor function */
    //     [&] (size_t dn) {
    //       if (!buildProgress) return true;
    //       const size_t n = progress.fetch_add(dn)+dn;
    //       const double f = std::min(1.0,double(n)/double(primitiveCount));
    //       return buildProgress(userPtr,f);
    //     },

    //     (PrimRef*)prims,pinfo,*arguments);

    //   bvh->allocator.cleanup();
    //   return root;
    // }

    static __forceinline const std::pair<CentGeomBBox3fa,unsigned int>
    mergePair(const std::pair<CentGeomBBox3fa,unsigned int>& a,
              const std::pair<CentGeomBBox3fa,unsigned int>& b)
    {
      CentGeomBBox3fa centBounds = CentGeomBBox3fa::merge2(a.first,b.first);
      unsigned int maxGeomID = max(a.second,b.second);
      return std::pair<CentGeomBBox3fa,unsigned int>(centBounds,maxGeomID);
    }
  }
}

using namespace embree;
using namespace embree::isa;

RTC_NAMESPACE_BEGIN

    RTC_API RTCBVH rtcNewBVH(RTCDevice device)
    {
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcNewAllocator);
      RTC_VERIFY_HANDLE(device);
      BVH* bvh = new BVH((Device*)device);
      return (RTCBVH) bvh->refInc();
      RTC_CATCH_END((Device*)device);
      return nullptr;
    }

    RTC_API void* rtcThreadLocalAlloc(RTCThreadLocalAllocator localAllocator, size_t bytes, size_t align)
    {
      FastAllocator::CachedAllocator* alloc = (FastAllocator::CachedAllocator*) localAllocator;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcThreadLocalAlloc);
      return alloc->malloc0(bytes,align);
      RTC_CATCH_END(alloc->alloc->getDevice());
      return nullptr;
    }

    RTC_API void rtcMakeStaticBVH(RTCBVH hbvh)
    {
      BVH* bvh = (BVH*) hbvh;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcStaticBVH);
      RTC_VERIFY_HANDLE(hbvh);
      // bvh->morton_src.clear();
      // bvh->morton_tmp.clear();
      RTC_CATCH_END(bvh->device);
    }

    RTC_API void rtcRetainBVH(RTCBVH hbvh)
    {
      BVH* bvh = (BVH*) hbvh;
      Device* device = bvh ? bvh->device : nullptr;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcRetainBVH);
      RTC_VERIFY_HANDLE(hbvh);
      bvh->refInc();
      RTC_CATCH_END(device);
    }

RTC_API void rtcReleaseBVH(RTCBVH hbvh)
{
  BVH* bvh = (BVH*) hbvh;
  Device* device = bvh ? bvh->device : nullptr;
  RTC_CATCH_BEGIN;
  RTC_TRACE(rtcReleaseBVH);
  RTC_VERIFY_HANDLE(hbvh);
  bvh->refDec();
  RTC_CATCH_END(device);
}

RTC_NAMESPACE_END
