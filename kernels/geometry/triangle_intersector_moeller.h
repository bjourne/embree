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

#include "triangle.h"

/*! This intersector implements a modified version of the Moeller
 *  Trumbore intersector from the paper "Fast, Minimum Storage
 *  Ray-Triangle Intersection". In contrast to the paper we
 *  precalculate some factors and factor the calculations differently
 *  to allow precalculating the cross product e1 x e2. The resulting
 *  algorithm is similar to the fastest one of the paper "Optimizing
 *  Ray-Triangle Intersection via Automated Search". */

namespace embree
{
  namespace isa
  {
    template<int M>
    struct MoellerTrumboreHitM
    {
      __forceinline MoellerTrumboreHitM() {}

      __forceinline MoellerTrumboreHitM(const vbool<M>& valid,
                                        const vfloat<M>& U,
                                        const vfloat<M>& V,
                                        const vfloat<M>& T,
                                        const vfloat<M>& absDen,
                                        const Vec3vf<M>& Ng)
        : U(U), V(V), T(T), absDen(absDen), valid(valid), vNg(Ng) {}

      __forceinline void finalize()
      {
        const vfloat<M> rcpAbsDen = rcp(absDen);
        vt = T * rcpAbsDen;
        vu = U * rcpAbsDen;
        vv = V * rcpAbsDen;
      }

      __forceinline Vec3fa Ng(const size_t i) const { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }

    public:
      vfloat<M> U;
      vfloat<M> V;
      vfloat<M> T;
      vfloat<M> absDen;

    public:
      vbool<M> valid;
      vfloat<M> vu;
      vfloat<M> vv;
      vfloat<M> vt;
      Vec3vf<M> vNg;
    };

    template<int M>
    struct MoellerTrumboreIntersector1
    {
      __forceinline MoellerTrumboreIntersector1() {}

      __forceinline MoellerTrumboreIntersector1(const Ray& ray, const void* ptr) {}

    };

    template<int K>
    struct MoellerTrumboreHitK
    {
      __forceinline MoellerTrumboreHitK() {}
      __forceinline MoellerTrumboreHitK(const vfloat<K>& U,
                                        const vfloat<K>& V,
                                        const vfloat<K>& T,
                                        const vfloat<K>& absDen,
                                        const Vec3vf<K>& Ng)
        : U(U), V(V), T(T), absDen(absDen), Ng(Ng) {}

      __forceinline std::tuple<vfloat<K>,vfloat<K>,vfloat<K>,Vec3vf<K>>
      finalizeK () const
      {
        const vfloat<K> rcpAbsDen = rcp(absDen);
        const vfloat<K> t = T * rcpAbsDen;
        const vfloat<K> u = U * rcpAbsDen;
        const vfloat<K> v = V * rcpAbsDen;
        return std::make_tuple(u,v,t,Ng);
      }

    private:
      const vfloat<K> U;
      const vfloat<K> V;
      const vfloat<K> T;
      const vfloat<K> absDen;
      const Vec3vf<K> Ng;
    };

    template<int M, int K>
    struct MoellerTrumboreIntersectorK
    {
      __forceinline
      MoellerTrumboreIntersectorK(const vbool<K>& valid,
                                  const RayK<K>& ray)
      {
      }
    };
  }
}
