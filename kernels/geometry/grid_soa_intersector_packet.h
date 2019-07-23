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
#include "../common/ray.h"
#include "triangle_intersector_pluecker.h"

namespace embree
{
  namespace isa
  {
    template<int K>
      struct MapUV0
    {
      const float* const grid_uv;
      size_t ofs00, ofs01, ofs10, ofs11;

      __forceinline MapUV0(const float* const grid_uv, size_t ofs00, size_t ofs01, size_t ofs10, size_t ofs11)
        : grid_uv(grid_uv), ofs00(ofs00), ofs01(ofs01), ofs10(ofs10), ofs11(ofs11) {}

      __forceinline void operator() (vfloat<K>& u, vfloat<K>& v) const {
        const vfloat<K> uv00(grid_uv[ofs00]);
        const vfloat<K> uv01(grid_uv[ofs01]);
        const vfloat<K> uv10(grid_uv[ofs10]);
        const vfloat<K> uv11(grid_uv[ofs11]);
        const Vec2vf<K> uv0 = GridSOA::decodeUV(uv00);
        const Vec2vf<K> uv1 = GridSOA::decodeUV(uv01);
        const Vec2vf<K> uv2 = GridSOA::decodeUV(uv10);
        const Vec2vf<K> uv = madd(u,uv1,madd(v,uv2,(1.0f-u-v)*uv0));
        u = uv[0]; v = uv[1];
      }
    };

    template<int K>
      struct MapUV1
    {
      const float* const grid_uv;
      size_t ofs00, ofs01, ofs10, ofs11;

      __forceinline MapUV1(const float* const grid_uv, size_t ofs00, size_t ofs01, size_t ofs10, size_t ofs11)
        : grid_uv(grid_uv), ofs00(ofs00), ofs01(ofs01), ofs10(ofs10), ofs11(ofs11) {}

      __forceinline void operator() (vfloat<K>& u, vfloat<K>& v) const {
        const vfloat<K> uv00(grid_uv[ofs00]);
        const vfloat<K> uv01(grid_uv[ofs01]);
        const vfloat<K> uv10(grid_uv[ofs10]);
        const vfloat<K> uv11(grid_uv[ofs11]);
        const Vec2vf<K> uv0 = GridSOA::decodeUV(uv10);
        const Vec2vf<K> uv1 = GridSOA::decodeUV(uv01);
        const Vec2vf<K> uv2 = GridSOA::decodeUV(uv11);
        const Vec2vf<K> uv = madd(u,uv1,madd(v,uv2,(1.0f-u-v)*uv0));
        u = uv[0]; v = uv[1];
      }
    };
  }
}
