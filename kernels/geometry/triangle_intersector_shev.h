#pragma once

template<int K, int M>
static __forceinline bool
isect(const Vec3vf<K>& o,
      const Vec3vf<K>& d,
      const vfloat<K>& tn, const vfloat<K>& tf,
      size_t i,
      const vbool<K>& valid0,
      MTHit<K>& hit,
      const TriangleM<M>& tri) {

  vfloat<K> np = vfloat<K>(tri.np[i]);
  vfloat<K> nu = vfloat<K>(tri.nu[i]);
  vfloat<K> nv = vfloat<K>(tri.nv[i]);
  vfloat<K> pu = vfloat<K>(tri.pu[i]);
  vfloat<K> pv = vfloat<K>(tri.pv[i]);
  vfloat<K> dett, det, du, dv;
  if (tri.ci[i] == 0) {
    dett = np - (o.y * nu + o.z * nv + o.x);
    det = d.y * nu + d.z * nv + d.x;
    du = d.y * dett - (pu - o.y) * det;
    dv = d.z * dett - (pv - o.z) * det;
  } else if (tri.ci[i] == 1) {
    dett = np - (o.x * nu + o.z * nv + o.y);
    det = d.x * nu + d.z * nv + d.y;
    du = d.x * dett - (pu - o.x) * det;
    dv = d.z * dett - (pv - o.z) * det;
  } else {
    dett = np - (o.x * nu + o.y * nv + o.z);
    det = d.x * nu + d.y * nv + d.z;
    du = d.x * dett - (pu - o.x) * det;
    dv = d.y * dett - (pv - o.y) * det;
  }

  vfloat<K> u = tri.e2v[i] * du - tri.e2u[i] * dv;
  vfloat<K> v = tri.e1u[i] * dv - tri.e1v[i] * du;
  vfloat<K> p0 = ((det - u - v) ^ u) | (u ^ v);

  vbool<K> valid = asInt(signmsk(p0)) == vint<K>(zero);
  if (likely(none(valid))) {
    return false;
  }

  vfloat<K> rdet = rcp(det);
  vfloat<K> t = dett * rdet;
  valid &= (tn < t) & (t <= tf);
  if (likely(none(valid))) {
    return false;
  }

  // Reconstruct the normal
  float x, y, z;
  if (tri.ci[i] == 0) {
    x = 1.0, y = nu[i], z = nv[i];
  } else if (tri.ci[i] == 1) {
    x = nu[i], y = 1.0, z = nv[i];
  } else {
    x = nu[i], y = nv[i], z = 1.0;
  }
  Vec3vf<K> ng = Vec3vf<K>(vfloat<K>(x), vfloat<K>(y), vfloat<K>(z));
  new (&hit) MTHit<K>(valid, u * rdet, v * rdet, t, ng);
  return true;
}

template<int M, int K>
static __forceinline bool
intersectKRaysMTris(const Vec3vf<K>& o, const Vec3vf<K>& d,
                    const vfloat<K>& tn, const vfloat<K>& tf,
                    size_t i,
                    const vbool<K>& valid0,
                    MTHit<K>& hit,
                    const TriangleM<M>& tri)
{
  return isect(o, d, tn, tf, i, valid0, hit, tri);
}

/*! Intersect 1 ray with one of M triangles. */
template<int M>
static __forceinline bool
intersect1RayMTris(Vec3vf<M> o, Vec3vf<M> d,
                   vfloat<M> tn, vfloat<M> tf,
                   const TriangleM<M>& tri, MTHit<M>& hit)
{
  assert(false);
  return false;
}
