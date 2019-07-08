#pragma once

#include "subdivpatch1.h"
#include "grid_soa.h"
//#include "grid_soa_intersector1.h"
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

}
}
