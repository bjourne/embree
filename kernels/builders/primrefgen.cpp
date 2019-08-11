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

#include "primrefgen.h"

#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    PrimInfo
    createPrimRefArray(Geometry* geometry,
                       mvector<PrimRef>& prims,
                       BuildProgressMonitor& progressMonitor)
    {
      printf("createPrimRefArray\n");
      ParallelPrefixSumState<PrimInfo> pstate;

      /* first try */
      progressMonitor(0);
      PrimInfo pinfo = parallel_prefix_sum( pstate, size_t(0), geometry->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo {
          return geometry->createPrimRefArray(prims,r,r.begin());
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_prefix_sum( pstate, size_t(0), geometry->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo {
          return geometry->createPrimRefArray(prims,r,base.size());
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }

    PrimInfo createPrimRefArray(Scene* scene, Geometry::GTypeMask types, bool mblur, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator2 iter(scene,types,mblur);

      /* first try */
      progressMonitor(0);
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum0( pstate, iter, PrimInfo(empty), [&](Geometry* mesh, const range<size_t>& r, size_t k) -> PrimInfo {
          return mesh->createPrimRefArray(prims,r,k);
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        progressMonitor(0);
        pinfo = parallel_for_for_prefix_sum1( pstate, iter, PrimInfo(empty), [&](Geometry* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo {
            return mesh->createPrimRefArray(prims,r,base.size());
          }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }
  }
}
