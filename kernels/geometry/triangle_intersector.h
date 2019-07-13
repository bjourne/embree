#pragma once

#include "triangle.h"
#include "triangle_intersector_moeller.h"

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
/*! Intersects M triangles with 1 ray */
template<int M, int Mx, bool filter>
struct TriangleMIntersector1Moeller
{
    typedef TriangleM<M> Primitive;
    typedef MoellerTrumboreIntersector1<Mx> Precalculations;

    static __forceinline void
    intersect_mt(RayHit& ray,
                 IntersectContext* context,
                 const TriangleM<M>& tri)
    {
        vbool<M> valid = true;

        Vec3<vfloat<M>> o = Vec3<vfloat<M>>(ray.org);
        Vec3<vfloat<M>> d = Vec3<vfloat<M>>(ray.dir);

        Vec3<vfloat<M>> v0 = tri.v0;
        Vec3<vfloat<M>> e1 = -tri.e1;
        Vec3<vfloat<M>> e2 = tri.e2;

        Vec3<vfloat<M>> pvec = cross(d, e2);
        vfloat<M> det = dot(e1, pvec);
        valid &= det != vfloat<M>(zero);
        if (likely(none(valid)))
            return;
        vfloat<M> inv_det = rcp(det);
        Vec3<vfloat<M>> tvec = o - v0;
        vfloat<M> vu = dot(tvec, pvec) * inv_det;
        valid &= (vu >= 0) & (vu <= 1.0);
        if (likely(none(valid)))
            return;
        Vec3<vfloat<M>> qvec = cross(tvec, e1);
        vfloat<M> vv = dot(d, qvec) * inv_det;
        valid &= (0 < vv) & (vv + vu <= 1.0);
        if (likely(none(valid)))
            return;
        vfloat<M> vt = dot(e2, qvec) * inv_det;
        vfloat<M> tnear = vfloat<M>(ray.tnear());
        vfloat<M> tfar = vfloat<M>(ray.tfar);
        valid &= (tnear < vt) & (vt <= tfar);
        if (likely(none(valid)))
            return;

        Scene* scene = context->scene;
        vbool<Mx> valid2 = valid;
        if (Mx > M)
            valid2 &= (1 << M) - 1;

        size_t i = select_min(valid2, vt);
        unsigned int geomID = tri.geomIDs[i];

        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        goto entry;
        while (true)
        {
            if (unlikely(none(valid2)))
                return;
            i = select_min(valid2, vt);

            geomID = tri.geomIDs[i];
        entry:
            Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
                clear(valid2,i);
                continue;
            }
#endif
            break;
        }
#endif
        /* update hit information */
        Vec3<vfloat<M>> tri_Ng = cross(tri.e2, tri.e1);
        ray.tfar = vt[i];
        ray.Ng.x = tri_Ng.x[i];
        ray.Ng.y = tri_Ng.y[i];
        ray.Ng.z = tri_Ng.z[i];

        ray.u = vu[i];
        ray.v = vv[i];

        ray.primID = tri.primIDs[i];
        ray.geomID = geomID;
        ray.instID = context->instID;
    }

    static __forceinline void
    intersect_sf01(RayHit& ray,
                   IntersectContext* context,
                   const TriangleM<M>& tri)
    {
        vbool<M> valid = true;
        Vec3<vfloat<M>> O = Vec3<vfloat<M>>(ray.org);
        Vec3<vfloat<M>> D = Vec3<vfloat<M>>(ray.dir);

        Vec3<vfloat<M>> v1 = tri.v0 - tri.e1;
        Vec3<vfloat<M>> v2 = tri.e2 + tri.v0;

        Vec3<vfloat<M>> v0o = tri.v0 - O;
        Vec3<vfloat<M>> v1o = v1 - O;
        Vec3<vfloat<M>> v2o = v2 - O;

        vfloat<M> w2 = dot(D, cross(v1o, v0o));
        vfloat<M> w0 = dot(D, cross(v2o, v1o));
        vbool<M> s2 = w2 >= vfloat<M>(zero);
        vbool<M> s0 = w0 >= vfloat<M>(zero);

        valid &= s0 == s2;
        if (likely(none(valid))) {
            return;
        }
        vfloat<M> w1 = dot(D, cross(v0o, v2o));
        vbool<M> s1 = w1 >= vfloat<M>(zero);
        valid &= s2 == s1;
        if (likely(none(valid))) {
            return;
        }

        Vec3<vfloat<M>> tri_Ng = cross(tri.e2, tri.e1);
        vfloat<M> den = dot(tri_Ng, D);
        vfloat<M> T = dot(tri_Ng, v0o) * rcp(den);
        valid &= (vfloat<M>(ray.tnear()) < T) & (T <= vfloat<M>(ray.tfar));
        if (likely(none(valid)))
            return;

        // Epilog
        Scene* scene = context->scene;
        vbool<Mx> valid2 = valid;
        if (Mx > M)
            valid2 &= (1 << M) - 1;

        vfloat<M> vt = T;
        vfloat<M> vu = w1 * rcp(w0 + w1 + w2);
        vfloat<M> vv = w2 * rcp(w0 + w1 + w2);

        size_t i = select_min(valid2, vt);
        unsigned int geomID = tri.geomIDs[i];
        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        goto entry;
        while (true)
        {
            if (unlikely(none(valid2)))
                return;
            i = select_min(valid2, vt);

            geomID = tri.geomIDs[i];
        entry:
            Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
                clear(valid2,i);
                continue;
            }
#endif
            break;
        }
#endif
        /* update hit information */
        ray.tfar = vt[i];
        ray.Ng.x = tri_Ng.x[i];
        ray.Ng.y = tri_Ng.y[i];
        ray.Ng.z = tri_Ng.z[i];

        ray.u = vu[i];
        ray.v = vv[i];

        ray.primID = tri.primIDs[i];
        ray.geomID = geomID;
        ray.instID = context->instID;
    }

    /*! Intersect a ray with the M triangles and updates the hit. */
    static __forceinline void
    intersect_orig(RayHit& ray,
                   IntersectContext* context,
                   const TriangleM<M>& tri)
    {
        vbool<M> valid = true;
        Vec3<vfloat<M>> tri_Ng = cross(tri.e2, tri.e1);

        Vec3<vfloat<M>> O = Vec3<vfloat<M>>(ray.org);
        Vec3<vfloat<M>> D = Vec3<vfloat<M>>(ray.dir);
        Vec3<vfloat<M>> C = Vec3<vfloat<M>>(tri.v0) - O;
        Vec3<vfloat<M>> R = cross(C, D);

        vfloat<M> den = dot(tri_Ng, D);
        vfloat<M> absDen = abs(den);
        vfloat<M> sgnDen = signmsk(den);

        /* perform edge tests */
        vfloat<M> U = dot(R, tri.e2) ^ sgnDen;
        vfloat<M> V = dot(R, tri.e1) ^ sgnDen;

        // No backface culling
        valid &= (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
        if (likely(none(valid)))
            return;

        /* perform depth test */
        vfloat<M> T = dot(Vec3vf<M>(tri_Ng), C) ^ sgnDen;
        valid &= (absDen*vfloat<M>(ray.tnear()) < T) & (T <= absDen*vfloat<M>(ray.tfar));
        if (likely(none(valid)))
            return;

        Scene* scene = context->scene;
        vbool<Mx> valid2 = valid;
        if (Mx > M)
            valid2 &= (1 << M) - 1;

        // Trick to avoid dividing to early.
        vfloat<M> rcpAbsDen = rcp(absDen);
        vfloat<M> vt = T * rcpAbsDen;
        vfloat<M> vu = U * rcpAbsDen;
        vfloat<M> vv = V * rcpAbsDen;

        size_t i = select_min(valid2, vt);
        unsigned int geomID = tri.geomIDs[i];

        /* intersection filter test */
#if defined(EMBREE_FILTER_FUNCTION) || defined(EMBREE_RAY_MASK)
        goto entry;
        while (true)
        {
            if (unlikely(none(valid2)))
                return;
            i = select_min(valid2, vt);

            geomID = tri.geomIDs[i];
        entry:
            Geometry* geometry MAYBE_UNUSED = scene->get(geomID);

#if defined(EMBREE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
                clear(valid2,i);
                continue;
            }
#endif
            break;
        }
#endif
        /* update hit information */
        ray.tfar = vt[i];
        ray.Ng.x = tri_Ng.x[i];
        ray.Ng.y = tri_Ng.y[i];
        ray.Ng.z = tri_Ng.z[i];

        ray.u = vu[i];
        ray.v = vv[i];

        ray.primID = tri.primIDs[i];
        ray.geomID = geomID;
        ray.instID = context->instID;
    }

    static __forceinline void
    intersect(RayHit& ray,
              IntersectContext* context,
              const TriangleM<M>& tri)
    {
        STAT3(normal.trav_prims,1,1,1);
        intersect_orig(ray, context, tri);
    }

    /*! Test if the ray is occluded by one of M triangles. */
    static __forceinline bool
    occluded(Ray& ray,
             IntersectContext* context,
             const TriangleM<M>& tri)
    {
        assert(false);
        return true;
    }
};


}
}
