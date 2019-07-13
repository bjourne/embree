#pragma once

#include "node_intersector.h"

namespace embree
{
namespace isa
{
// Ray packet structure used in hybrid traversal

template<int K, bool robust>
struct TravRayK;

/* Fast variant */
template<int K>
struct TravRayK<K, false>
{
      __forceinline TravRayK() {}

      __forceinline TravRayK(const Vec3vf<K>& ray_org, const Vec3vf<K>& ray_dir, int N)
      {
        init(ray_org, ray_dir, N);
      }

      __forceinline TravRayK(const Vec3vf<K>& ray_org, const Vec3vf<K>& ray_dir, const vfloat<K>& ray_tnear, const vfloat<K>& ray_tfar, int N)
      {
        init(ray_org, ray_dir, N);
        tnear = ray_tnear;
        tfar = ray_tfar;
      }

      __forceinline void init(const Vec3vf<K>& ray_org, const Vec3vf<K>& ray_dir, int N)
      {
        org = ray_org;
        dir = ray_dir;
        rdir = rcp_safe(ray_dir);
#if defined(__AVX2__)
        org_rdir = org * rdir;
#endif

        if (N)
        {
          const int size = sizeof(float)*N;
          nearXYZ.x = select(rdir.x >= 0.0f, vint<K>(0*size), vint<K>(1*size));
          nearXYZ.y = select(rdir.y >= 0.0f, vint<K>(2*size), vint<K>(3*size));
          nearXYZ.z = select(rdir.z >= 0.0f, vint<K>(4*size), vint<K>(5*size));
        }
      }

      Vec3vf<K> org;
      Vec3vf<K> dir;
      Vec3vf<K> rdir;
#if defined(__AVX2__)
      Vec3vf<K> org_rdir;
#endif
      Vec3vi<K> nearXYZ;
      vfloat<K> tnear;
      vfloat<K> tfar;
    };

    template<int K>
    using TravRayKFast = TravRayK<K, false>;

    /* Robust variant */
    template<int K>
    struct TravRayK<K, true>
    {
      __forceinline TravRayK() {}

      __forceinline TravRayK(const Vec3vf<K>& ray_org, const Vec3vf<K>& ray_dir, int N)
      {
        init(ray_org, ray_dir, N);
      }

      __forceinline TravRayK(const Vec3vf<K>& ray_org, const Vec3vf<K>& ray_dir, const vfloat<K>& ray_tnear, const vfloat<K>& ray_tfar, int N)
      {
        init(ray_org, ray_dir, N);
        tnear = ray_tnear;
        tfar = ray_tfar;
      }

      __forceinline void init(const Vec3vf<K>& ray_org, const Vec3vf<K>& ray_dir, int N)
      {
        org = ray_org;
        dir = ray_dir;
        rdir = vfloat<K>(1.0f)/(zero_fix(ray_dir));

        if (N)
        {
          const int size = sizeof(float)*N;
          nearXYZ.x = select(rdir.x >= 0.0f, vint<K>(0*size), vint<K>(1*size));
          nearXYZ.y = select(rdir.y >= 0.0f, vint<K>(2*size), vint<K>(3*size));
          nearXYZ.z = select(rdir.z >= 0.0f, vint<K>(4*size), vint<K>(5*size));
        }
      }

      Vec3vf<K> org;
      Vec3vf<K> dir;
      Vec3vf<K> rdir;
      Vec3vi<K> nearXYZ;
      vfloat<K> tnear;
      vfloat<K> tfar;
    };

template<int K>
    using TravRayKRobust = TravRayK<K, true>;

}
}
