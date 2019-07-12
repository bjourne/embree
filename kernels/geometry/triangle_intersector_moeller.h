#pragma once

#include "triangle.h"

namespace embree
{
namespace isa
{

template<int M>
struct MoellerTrumboreIntersector1
{
    __forceinline MoellerTrumboreIntersector1()
    {
    }

    __forceinline MoellerTrumboreIntersector1(const Ray& ray,
                                              const void* ptr)
    {
    }
};


}
}
