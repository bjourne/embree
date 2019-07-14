#pragma once

#include "primitive.h"

namespace embree
{
/* Precalculated representation for M triangles. Stores for each
   triangle a base vertex, two edges, and the geometry normal to
   speed up intersection calculations

   TODO: Check if this is true - I can't find the normals.
*/
template<int M>
struct TriangleM
{
public:
    struct Type : public PrimitiveType
    {
        const char* name() const;
        size_t sizeActive(const char* This) const;
        size_t sizeTotal(const char* This) const;
        size_t getBytes(const char* This) const;
    };
    static Type type;

public:

    /* Returns maximum number of stored triangles */
    static __forceinline size_t max_size()
    {
        return M;
    }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N)
    {
        return (N+max_size()-1)/max_size();
    }

public:

    /* Construction from vertices and IDs */
    __forceinline TriangleM(const Vec3vf<M>& v0,
                            const Vec3vf<M>& v1,
                            const Vec3vf<M>& v2,
                            const vuint<M>& geomIDs,
                            const vuint<M>& primIDs)
      : v0(v0), e1(v0 - v1), e2(v2 - v0),
        geomIDs(geomIDs), primIDs(primIDs)
    {
        Vec3<vfloat<M>> e1p = v1 - v0;
        n0 = cross(e1p, e2);
        d0 = dot(n0, v0);
        vfloat<M> inv_denom = rcp(dot(n0, n0));
        n1 = cross(e2, n0) * inv_denom;
        d1 = -dot(n1, v0);
        n2 = cross(n0, e1p) * inv_denom;
        d2 = -dot(n2, v0);
    }

    /* Returns a mask that tells which triangles are valid */
    __forceinline vbool<M> valid() const
    {
        return geomIDs != vuint<M>(-1);
    }

    /* Returns true if the specified triangle is valid */
    __forceinline bool valid(const size_t i) const
    {
        assert(i<M);
        return geomIDs[i] != -1;
    }

    /* Returns the number of stored triangles */
    __forceinline size_t size() const {
        return bsf(~movemask(valid()));
    }

    __forceinline unsigned int geomID(const size_t i) const
    {
        assert(i<M); return geomIDs[i];
    }

    __forceinline unsigned int primID(const size_t i) const
    {
        assert(i<M); return primIDs[i];
    }

    /* Calculate the bounds of the triangle */
    __forceinline BBox3fa
    bounds() const
    {
        Vec3vf<M> p0 = v0;
        Vec3vf<M> p1 = v0-e1;
        Vec3vf<M> p2 = v0+e2;
        Vec3vf<M> lower = min(p0,p1,p2);
        Vec3vf<M> upper = max(p0,p1,p2);
        vbool<M> mask = valid();
        lower.x = select(mask,lower.x,vfloat<M>(pos_inf));
        lower.y = select(mask,lower.y,vfloat<M>(pos_inf));
        lower.z = select(mask,lower.z,vfloat<M>(pos_inf));
        upper.x = select(mask,upper.x,vfloat<M>(neg_inf));
        upper.y = select(mask,upper.y,vfloat<M>(neg_inf));
        upper.z = select(mask,upper.z,vfloat<M>(neg_inf));
        return BBox3fa(Vec3fa(reduce_min(lower.x),
                              reduce_min(lower.y),
                              reduce_min(lower.z)),
                       Vec3fa(reduce_max(upper.x),
                              reduce_max(upper.y),
                              reduce_max(upper.z)));
    }

    /* Non temporal store */
    __forceinline static void store_nt(TriangleM* dst,
                                       const TriangleM& src)
    {
        vfloat<M>::store_nt(&dst->v0.x,src.v0.x);
        vfloat<M>::store_nt(&dst->v0.y,src.v0.y);
        vfloat<M>::store_nt(&dst->v0.z,src.v0.z);
        vfloat<M>::store_nt(&dst->e1.x,src.e1.x);
        vfloat<M>::store_nt(&dst->e1.y,src.e1.y);
        vfloat<M>::store_nt(&dst->e1.z,src.e1.z);
        vfloat<M>::store_nt(&dst->e2.x,src.e2.x);
        vfloat<M>::store_nt(&dst->e2.y,src.e2.y);
        vfloat<M>::store_nt(&dst->e2.z,src.e2.z);
        vuint<M>::store_nt(&dst->geomIDs,src.geomIDs);
        vuint<M>::store_nt(&dst->primIDs,src.primIDs);

        // HH
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
    }

    /* Fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims,
                            size_t& begin, size_t end,
                            Scene* scene)
    {
        vuint<M> vgeomID = -1, vprimID = -1;
        Vec3vf<M> v0 = zero, v1 = zero, v2 = zero;

        for (size_t i=0; i < M && begin < end; i++, begin++)
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
        TriangleM::store_nt(this,TriangleM(v0,v1,v2,vgeomID,vprimID));
    }

    /* Updates the primitive */
    __forceinline BBox3fa update(TriangleMesh* mesh)
    {
        BBox3fa bounds = empty;
        vuint<M> vgeomID = -1, vprimID = -1;
        Vec3vf<M> v0 = zero, v1 = zero, v2 = zero;

        for (size_t i=0; i<M; i++)
        {
            if (unlikely(geomID(i) == -1)) break;
            const unsigned geomId = geomID(i);
            const unsigned primId = primID(i);
            const TriangleMesh::Triangle& tri = mesh->triangle(primId);
            const Vec3fa p0 = mesh->vertex(tri.v[0]);
            const Vec3fa p1 = mesh->vertex(tri.v[1]);
            const Vec3fa p2 = mesh->vertex(tri.v[2]);
            bounds.extend(merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2)));
            vgeomID [i] = geomId;
            vprimID [i] = primId;
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
        TriangleM::store_nt(this,TriangleM(v0,v1,v2,vgeomID,vprimID));
        return bounds;
    }

public:
    Vec3vf<M> v0;      // base vertex of the triangles
    Vec3vf<M> e1;      // 1st edge of the triangles (v0-v1)
    Vec3vf<M> e2;      // 2nd edge of the triangles (v2-v0)
    vuint<M> geomIDs; // geometry IDs
    vuint<M> primIDs; // primitive IDs

    Vec3vf<M> n0, n1, n2;
    vfloat<M> d0, d1, d2;
};

template<int M>
typename TriangleM<M>::Type TriangleM<M>::type;

typedef TriangleM<4> Triangle4;
}
