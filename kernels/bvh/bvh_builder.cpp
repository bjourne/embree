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

#include "bvh_builder.h"

namespace embree
{
  namespace isa
  {
    template<int N>
    typename BVHN<N>::NodeRef BVHNBuilderVirtual<N>::BVHNBuilderV::build(
      FastAllocator* allocator,
      BuildProgressMonitor& progressFunc,
      PrimRef* prims,
      const PrimInfo& pinfo,
      GeneralBVHBuilder::Settings settings)
    {
      auto createLeafFunc = [&] (const PrimRef* prims,
                                 const range<size_t>& set,
                                 const Allocator& alloc) -> NodeRef {
        return createLeaf(prims,set,alloc);
      };

      settings.branchingFactor = N;
      settings.maxDepth = BVH::maxBuildDepthLeaf;
      return BVHBuilderBinnedSAH::build<NodeRef>
        (FastAllocator::Create(allocator),typename BVH::AlignedNode::Create2(),typename BVH::AlignedNode::Set3(allocator,prims),createLeafFunc,progressFunc,prims,pinfo,settings);
    }

    template struct BVHNBuilderVirtual<4>;

#if defined(__AVX__)
    template struct BVHNBuilderVirtual<8>;
#endif
  }
}
