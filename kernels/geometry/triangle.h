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
#include "../common/scene.h"
#include "../common/primref.h"

#define ISECT_EMBREE    1
#define ISECT_HH        2
#define ISECT_SF01      3
#define ISECT_MT        4
#define ISECT_BW12      5
#define ISECT_BW9       6
#define ISECT_SHEV      7

#define ISECT_METHOD ISECT_SHEV

#if ISECT_METHOD == ISECT_EMBREE
#define ISECT_NAME "embree"
#elif ISECT_METHOD == ISECT_HH
#define ISECT_NAME "hh"
#elif ISECT_METHOD == ISECT_SF01
#define ISECT_NAME "sf01"
#elif ISECT_METHOD == ISECT_MT
#define ISECT_NAME "mt"
#elif ISECT_METHOD == ISECT_BW12
#define ISECT_NAME "bw12"
#elif ISECT_METHOD == ISECT_BW9
#define ISECT_NAME "bw9"
#elif ISECT_METHOD == ISECT_SHEV
#define ISECT_NAME "shev"
#else
#error "Wrong ISECT_METHOD!"
#endif

namespace embree
{
  template<int M>
  struct TriangleM
  {
  public:

    /* Returns maximum number of stored triangles */
    static __forceinline size_t
    max_size()
    {
      return M;
    }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t
    blocks(size_t N)
    {
      return (N + M - 1) / M;
    }

    /* Default constructor */
    __forceinline TriangleM() {}

    #if ISECT_METHOD == ISECT_SHEV

    vfloat<M> nu, nv, pu, pv, np, e1u, e1v, e2u, e2v;
    vint<M> ci;

    __forceinline static void
    do_store_nt(TriangleM* dst, const TriangleM& src)
    {
      vfloat<M>::store_nt(&dst->nu, src.nu);
      vfloat<M>::store_nt(&dst->nv, src.nv);
      vfloat<M>::store_nt(&dst->pu, src.pu);
      vfloat<M>::store_nt(&dst->pv, src.pv);
      vfloat<M>::store_nt(&dst->np, src.np);
      vfloat<M>::store_nt(&dst->e1u, src.e1u);
      vfloat<M>::store_nt(&dst->e1v, src.e1v);
      vfloat<M>::store_nt(&dst->e2u, src.e2u);
      vfloat<M>::store_nt(&dst->e2v, src.e2v);
      vint<M>::store_nt(&dst->ci, src.ci);
    }

    void
    init_shev(const Vec3vf<M>& v0,
              const Vec3vf<M>& v1,
              const Vec3vf<M>& v2) {
      Vec3vf<M> e1 = v1 - v0;
      Vec3vf<M> e2 = v2 - v0;
      Vec3vf<M> n = cross(e1, e2);
      for (int i = 0; i < M; i++) {
        int u, v, w;
        if (fabsf(n.x[i]) > fabsf(n.y[i]) && fabsf(n.x[i]) > fabsf(n.z[i])) {
          w = 0; u = 1; v = 2;
        } else if (fabsf(n.y[i]) > fabsf(n.z[i])) {
          w = 1; u = 0; v = 2;
        } else {
          w = 2; u = 0; v = 1;
        }
        float sign = w == 1 ? -1.0f : 1.0f;
        float nw = n[w][i];
        nu[i] = n[u][i] / nw;
        nv[i] = n[v][i] / nw;
        pu[i] = v0[u][i];
        pv[i] = v0[v][i];
        np[i] = nu[i] * pu[i] + nv[i] * pv[i] + v0[w][i];
        e1u[i] = sign * e1[u][i] / nw;
        e1v[i] = sign * e1[v][i] / nw;
        e2u[i] = sign * e2[u][i] / nw;
        e2v[i] = sign * e2[v][i] / nw;
        ci[i] = w;
      }
    }
    #elif ISECT_METHOD == ISECT_BW9

    __forceinline static void
    do_store_nt(TriangleM* dst, const TriangleM& src)
    {
      memcpy(dst->T, src.T, sizeof(src.T));
      vint<M>::store_nt(&dst->ci, src.ci);
      vfloat<M>::store_nt(&dst->ng.x, src.ng.x);
      vfloat<M>::store_nt(&dst->ng.y, src.ng.y);
      vfloat<M>::store_nt(&dst->ng.z, src.ng.z);
    }

    void
    init_bw9(const Vec3vf<M>& v0,
             const Vec3vf<M>& v1,
             const Vec3vf<M>& v2)
    {
      Vec3vf<M> e1 = v1 - v0;
      Vec3vf<M> e2 = v2 - v0;
      this->ng = cross(e1, e2);
      vfloat<M> nums = dot(v0, ng);
      for (int i = 0; i < M; i++) {
        float nx = ng.x[i], ny = ng.y[i], nz = ng.z[i];
        float v0x = v0.x[i], v0y = v0.y[i], v0z = v0.z[i];
        float v1x = v1.x[i], v1y = v1.y[i], v1z = v1.z[i];
        float v2x = v2.x[i], v2y = v2.y[i], v2z = v2.z[i];
        float e1x = e1.x[i], e1y = e1.y[i], e1z = e1.z[i];
        float e2x = e2.x[i], e2y = e2.y[i], e2z = e2.z[i];
        float num = nums[i];
        if (fabsf(nx) > fabsf(ny) && fabsf(nx) > fabsf(nz)) {
          float x1 = v1y * v0z - v1z * v0y;
          float x2 = v2y * v0z - v2z * v0y;
          T[i][0] =  e2z / nx;
          T[i][1] = -e2y / nx;
          T[i][2] =   x2 / nx;
          T[i][3] = -e1z / nx;
          T[i][4] =  e1y / nx;
          T[i][5] =  -x1 / nx;
          T[i][6] =   ny / nx;
          T[i][7] =   nz / nx;
          T[i][8] = -num / nx;
          this->ci[i] = 0;
        } else if (fabsf(ny) > fabsf(nz)) {
          float x1 = v1z * v0x - v1x * v0z;
          float x2 = v2z * v0x - v2x * v0z;
          T[i][0] = -e2z / ny;
          T[i][1] =  e2x / ny;
          T[i][2] =   x2 / ny;
          T[i][3] =  e1z / ny;
          T[i][4] = -e1x / ny;
          T[i][5] =  -x1 / ny;
          T[i][6] =   nx / ny;
          T[i][7] =   nz / ny;
          T[i][8] = -num / ny;
          this->ci[i] = 1;
        } else {
          float x1 = v1x * v0y - v1y * v0x;
          float x2 = v2x * v0y - v2y * v0x;
          T[i][0] =  e2y / nz;
          T[i][1] = -e2x / nz;
          T[i][2] =   x2 / nz;
          T[i][3] = -e1y / nz;
          T[i][4] =  e1x / nz;
          T[i][5] =  -x1 / nz;
          T[i][6] =   nx / nz;
          T[i][7] =   ny / nz;
          T[i][8] = -num / nz;
          this->ci[i] = 2;
        }
      }
    }
    #endif

    /* Construction from vertices and IDs */
    __forceinline
    TriangleM(const Vec3vf<M>& v0,
              const Vec3vf<M>& v1,
              const Vec3vf<M>& v2,
              const vuint<M>& geomIDs,
              const vuint<M>& primIDs)
    {
      this->geomIDs = geomIDs;
      this->primIDs = primIDs;

      #if ISECT_METHOD == ISECT_HH
      n0 = cross(v1 - v0, v2 - v0);
      d0 = dot(n0, v0);
      vfloat<M> rdenom = rcp(dot(n0, n0));
      n1 = cross(v2 - v0, n0) * rdenom;
      d1 = -dot(n1, v0);
      n2 = cross(n0, v1 - v0) * rdenom;
      d2 = -dot(n2, v0);
      #elif ISECT_METHOD == ISECT_EMBREE
      this->v0 = v0;
      this->e1 = v0 - v1;
      this->e2 = v2 - v0;
      #elif ISECT_METHOD == ISECT_SF01
      this->v0 = v0;
      this->e1 = v0 - v1;
      this->e2 = v2 - v0;
      #elif ISECT_METHOD == ISECT_MT
      this->v0 = v0;
      this->e1 = v1 - v0;
      this->e2 = v2 - v0;

      #elif ISECT_METHOD == ISECT_BW12

      Vec3vf<M> e1 = v1 - v0;
      Vec3vf<M> e2 = v2 - v0;
      Vec3vf<M> n = cross(e1, e2);
      vfloat<M> num = dot(v0, n);
      for (int i = 0; i < M; i++) {

        Vec3fa n0fa, n1fa, n2fa;
        float d0fa, d1fa, d2fa;

        float nx = n.x[i], ny = n.y[i], nz = n.z[i];
        float v0x = v0.x[i], v0y = v0.y[i], v0z = v0.z[i];
        float v1x = v1.x[i], v1y = v1.y[i], v1z = v1.z[i];
        float v2x = v2.x[i], v2y = v2.y[i], v2z = v2.z[i];
        float e1x = e1.x[i], e1y = e1.y[i], e1z = e1.z[i];
        float e2x = e2.x[i], e2y = e2.y[i], e2z = e2.z[i];

        if (std::fabs(nx) > std::fabs(ny) && std::fabs(nx) > std::fabs(nz)) {
          float x1 = v1y * v0z - v1z * v0y;
          float x2 = v2y * v0z - v2z * v0y;

          n0fa = Vec3fa(0,  e2z / nx, -e2y / nx);
          n1fa = Vec3fa(0, -e1z / nx,  e1y / nx);
          n2fa = Vec3fa(1,   ny / nx,   nz / nx);

          d0fa =      x2 / nx;
          d1fa =     -x1 / nx;
          d2fa = -num[i] / nx;
        } else if (std::fabs(ny) > std::fabs(nz)) {
          float x1 = v1z * v0x - v1x * v0z;
          float x2 = v2z * v0x - v2x * v0z;

          n0fa = Vec3fa(-e2z / ny, 0,  e2x / ny);
          n1fa = Vec3fa( e1z / ny, 0, -e1x / ny);
          n2fa = Vec3fa(  nx / ny, 1,   nz / ny);

          d0fa =      x2 / ny;
          d1fa =     -x1 / ny;
          d2fa = -num[i] / ny;
        } else {
          float x1 = v1x * v0y - v1y * v0x;
          float x2 = v2x * v0y - v2y * v0x;

          n0fa = Vec3fa( e2y / nz, -e2x / nz, 0);
          n1fa = Vec3fa(-e1y / nz,  e1x / nz, 0);
          n2fa = Vec3fa(  nx / nz,   ny / nz, 1);

          d0fa =      x2 / nz;
          d1fa =     -x1 / nz;
          d2fa = -num[i] / nz;
        }
        n0.x[i] = n0fa.x; n0.y[i] = n0fa.y; n0.z[i] = n0fa.z;
        n1.x[i] = n1fa.x; n1.y[i] = n1fa.y; n1.z[i] = n1fa.z;
        n2.x[i] = n2fa.x; n2.y[i] = n2fa.y; n2.z[i] = n2fa.z;
        d0[i] = d0fa; d1[i] = d1fa; d2[i] = d2fa;
      }
      this->ng = cross(v1 - v0, v2 - v0);

      #elif ISECT_METHOD == ISECT_BW9
      init_bw9(v0, v1, v2);
      #elif ISECT_METHOD == ISECT_SHEV
      init_shev(v0, v1, v2);
      #else
      #error "Wrong ISECT_METHOD!"
      #endif
    }

    /* Returns a mask that tells which triangles are valid */
    __forceinline vbool<M>
    valid() const
    {
      return geomIDs != vuint<M>(-1);
    }

    /* Returns the number of stored triangles */
    __forceinline size_t size() const
    {
      return bsf(~movemask(valid()));
    }

    /* Returns the geometry IDs */
    __forceinline unsigned int
    geomID(const size_t i) const
    {
      assert(i<M);
      return geomIDs[i];
    }

    /* Returns the primitive IDs */
    __forceinline unsigned int
    primID(const size_t i) const
    {
      assert(i<M);
      return primIDs[i];
    }

    /* Non temporal store */
    __forceinline static void
    store_nt(TriangleM* dst, const TriangleM& src)
    {
      vuint<M>::store_nt(&dst->geomIDs, src.geomIDs);
      vuint<M>::store_nt(&dst->primIDs, src.primIDs);

      #if ISECT_METHOD == ISECT_HH
      vfloat<M>::store_nt(&dst->n0.x, src.n0.x);
      vfloat<M>::store_nt(&dst->n0.y, src.n0.y);
      vfloat<M>::store_nt(&dst->n0.z, src.n0.z);
      vfloat<M>::store_nt(&dst->n1.x, src.n1.x);
      vfloat<M>::store_nt(&dst->n1.y, src.n1.y);
      vfloat<M>::store_nt(&dst->n1.z, src.n1.z);
      vfloat<M>::store_nt(&dst->n2.x, src.n2.x);
      vfloat<M>::store_nt(&dst->n2.y, src.n2.y);
      vfloat<M>::store_nt(&dst->n2.z, src.n2.z);
      vfloat<M>::store_nt(&dst->d0, src.d0);
      vfloat<M>::store_nt(&dst->d1, src.d1);
      vfloat<M>::store_nt(&dst->d2, src.d2);
      #elif ISECT_METHOD == ISECT_BW12
      vfloat<M>::store_nt(&dst->n0.x, src.n0.x);
      vfloat<M>::store_nt(&dst->n0.y, src.n0.y);
      vfloat<M>::store_nt(&dst->n0.z, src.n0.z);
      vfloat<M>::store_nt(&dst->n1.x, src.n1.x);
      vfloat<M>::store_nt(&dst->n1.y, src.n1.y);
      vfloat<M>::store_nt(&dst->n1.z, src.n1.z);
      vfloat<M>::store_nt(&dst->n2.x, src.n2.x);
      vfloat<M>::store_nt(&dst->n2.y, src.n2.y);
      vfloat<M>::store_nt(&dst->n2.z, src.n2.z);
      vfloat<M>::store_nt(&dst->ng.x, src.ng.x);
      vfloat<M>::store_nt(&dst->ng.y, src.ng.y);
      vfloat<M>::store_nt(&dst->ng.z, src.ng.z);
      vfloat<M>::store_nt(&dst->d0, src.d0);
      vfloat<M>::store_nt(&dst->d1, src.d1);
      vfloat<M>::store_nt(&dst->d2, src.d2);
      #elif ISECT_METHOD == ISECT_BW9
      do_store_nt(dst, src);
      #elif ISECT_METHOD == ISECT_SHEV
      do_store_nt(dst, src);
      #elif ISECT_METHOD == ISECT_EMBREE || \
        ISECT_METHOD == ISECT_SF01 || \
        ISECT_METHOD == ISECT_MT
      vfloat<M>::store_nt(&dst->v0.x, src.v0.x);
      vfloat<M>::store_nt(&dst->v0.y, src.v0.y);
      vfloat<M>::store_nt(&dst->v0.z, src.v0.z);
      vfloat<M>::store_nt(&dst->e1.x, src.e1.x);
      vfloat<M>::store_nt(&dst->e1.y, src.e1.y);
      vfloat<M>::store_nt(&dst->e1.z, src.e1.z);
      vfloat<M>::store_nt(&dst->e2.x, src.e2.x);
      vfloat<M>::store_nt(&dst->e2.y, src.e2.y);
      vfloat<M>::store_nt(&dst->e2.z, src.e2.z);
      #else
      #error "Wrong ISECT_METHOD!"
      #endif
    }

    /* Fill triangle from triangle list */
    __forceinline void
    fill(const PrimRef* prims,
         size_t& begin,
         size_t end,
         Scene* scene)
    {
      vuint<M> vgeomID = -1, vprimID = -1;
      Vec3vf<M> v0 = zero, v1 = zero, v2 = zero;
      for (size_t i=0; i<M && begin<end; i++, begin++)
      {
        const PrimRef& prim = prims[begin];
        const unsigned geomID = prim.geomID();
        const unsigned primID = prim.primID();
        const TriangleMesh* __restrict__ const mesh =
          scene->get<TriangleMesh>(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        v0.x[i] = p0.x;
        v0.y[i] = p0.y;
        v0.z[i] = p0.z;
        v1.x[i] = p1.x;
        v1.y[i] = p1.y;
        v1.z[i] = p1.z;
        v2.x[i] = p2.x;
        v2.y[i] = p2.y;
        v2.z[i] = p2.z;
      }
      TriangleM::store_nt(this, TriangleM(v0, v1, v2, vgeomID, vprimID));
    }

  public:
    vuint<M> geomIDs;  // geometry IDs
    vuint<M> primIDs;  // primitive IDs

    #if ISECT_METHOD == ISECT_HH
    Vec3vf<M> n0, n1, n2;
    vfloat<M> d0, d1, d2;
    #elif ISECT_METHOD == ISECT_BW12
    Vec3vf<M> n0, n1, n2;
    vfloat<M> d0, d1, d2;
    Vec3vf<M> ng;
    #elif ISECT_METHOD == ISECT_BW9
    float T[M][9];
    vint<M> ci;
    //int ci[4];
    Vec3vf<M> ng;
    #elif ISECT_METHOD == ISECT_SHEV
    #elif ISECT_METHOD == ISECT_EMBREE || \
        ISECT_METHOD == ISECT_SF01 || \
        ISECT_METHOD == ISECT_MT
    Vec3vf<M> v0;      // base vertex of the triangles
    Vec3vf<M> e1;      // 1st edge of the triangles (v0-v1)
    Vec3vf<M> e2;      // 2nd edge of the triangles (v2-v0)
    #else
    #error "Wrong ISECT_METHOD!"
    #endif
  };
}
