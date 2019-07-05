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

#include "grid_soa.h"
#include "grid_soa_intersector1.h"
#include "grid_soa_intersector_packet.h"
#include "../common/ray.h"

namespace embree
{
  namespace isa
  {
    template<typename T>
      class SubdivPatch1Precalculations : public T
    {
    public:
      __forceinline SubdivPatch1Precalculations (const Ray& ray, const void* ptr)
        : T(ray,ptr) {}
    };

    template<int K, typename T>
      class SubdivPatch1PrecalculationsK : public T
    {
    public:
      __forceinline SubdivPatch1PrecalculationsK (const vbool<K>& valid, RayK<K>& ray)
        : T(valid,ray) {}
    };

typedef Grid1IntersectorK<4>  SubdivPatch1Intersector4;
typedef Grid1IntersectorK<8>  SubdivPatch1Intersector8;
typedef Grid1IntersectorK<16> SubdivPatch1Intersector16;

  }
}
