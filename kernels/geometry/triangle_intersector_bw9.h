#pragma once


template<int K, int M>
static __forceinline bool
isectAlgo(const Vec3vf<K>& o, const Vec3vf<K>& d,
          const vfloat<K>& tn, const vfloat<K>& tf,
          size_t i,
          const vbool<K>& valid0,
          MTHit<K>& hit,
          const TriangleM<M>& tri) {

  vfloat<K> t0 = vfloat<K>(tri.T[i][0]);
  vfloat<K> t1 = vfloat<K>(tri.T[i][1]);
  vfloat<K> t2 = vfloat<K>(tri.T[i][2]);
  vfloat<K> t3 = vfloat<K>(tri.T[i][3]);
  vfloat<K> t4 = vfloat<K>(tri.T[i][4]);
  vfloat<K> t5 = vfloat<K>(tri.T[i][5]);
  vfloat<K> t6 = vfloat<K>(tri.T[i][6]);
  vfloat<K> t7 = vfloat<K>(tri.T[i][7]);
  vfloat<K> t8 = vfloat<K>(tri.T[i][8]);

  vfloat<K> u, v, t;
  if (tri.ci[i] == 0) {
    vfloat<K> t_o = o.x + t6 * o.y + t7 * o.z + t8;
    vfloat<K> t_d = d.x + t6 * d.y + t7 * d.z;
    t = -t_o * rcp(t_d);
    Vec3vf<K> wr = o + d * t;
    u = t0 * wr.y + t1 * wr.z + t2;
    v = t3 * wr.y + t4 * wr.z + t5;
  } else if (tri.ci[i] == 1) {
    vfloat<K> t_o = t6 * o.x + o.y + t7 * o.z + t8;
    vfloat<K> t_d = t6 * d.x + d.y + t7 * d.z;
    t = -t_o * rcp(t_d);
    Vec3vf<K> wr = o + d * t;
    u = t0 * wr.x + t1 * wr.z + t2;
    v = t3 * wr.x + t4 * wr.z + t5;
  } else {
    vfloat<K> t_o = t6 * o.x + t7 * o.y + o.z + t8;
    vfloat<K> t_d = t6 * d.x + t7 * d.y + d.z;
    t = -t_o * rcp(t_d);
    Vec3vf<K> wr = o + d * t;
    u = t0 * wr.x + t1 * wr.y + t2;
    v = t3 * wr.x + t4 * wr.y + t5;
  }
  const vbool<K> valid = valid0
    & (u >= 0.0f) & (v >= 0.0f) & (u + v <= 1.0f)
    & (tn < t) & (t <= tf);
  if (likely(none(valid)))
    return false;
  new (&hit) MTHit<K>(valid, u, v, t,
                      broadcast<vfloat<K>>(tri.ng, i));
  return true;
}

// ~14.5 for crown
template<int M, int K>
static __forceinline bool
intersectKRaysMTris(const Vec3vf<K>& o, const Vec3vf<K>& d,
                    const vfloat<K>& tn, const vfloat<K>& tf,
                    size_t i,
                    const vbool<K>& valid0,
                    MTHit<K>& hit,
                    const TriangleM<M>& tri)
{
  return isectAlgo(o, d, tn, tf, i, valid0, hit, tri);
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
