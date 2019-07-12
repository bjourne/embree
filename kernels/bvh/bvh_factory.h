#pragma once

#include "../bvh/bvh.h"
#include "../common/isa.h"
#include "../common/accel.h"
#include "../common/scene.h"

namespace embree
{
  /*! BVH instantiations */
  class BVHFactory
  {
  public:
    enum class BuildVariant     { STATIC, DYNAMIC, HIGH_QUALITY };
    enum class IntersectVariant { FAST, ROBUST };
  };
}
