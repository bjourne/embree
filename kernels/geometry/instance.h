#pragma once

#include "primitive.h"
#include "../common/scene_instance.h"

namespace embree
{
struct InstancePrimitive
{
    struct Type : public PrimitiveType
    {
        const char* name() const;
        size_t sizeActive(const char* This) const;
        size_t sizeTotal(const char* This) const;
        size_t getBytes(const char* This) const;
    };
    static Type type;

public:

    /* primitive supports multiple time segments */
    static const bool singleTimeSegment = false;

    /* Returns maximum number of stored primitives */
    static __forceinline size_t max_size() { return 1; }

    /* Returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

  public:

    InstancePrimitive (const Instance* instance)
    : instance(instance) {}

    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene)
    {
      assert(end-i == 1);
      const PrimRef& prim = prims[i]; i++;
      const unsigned int geomID = prim.geomID();
      const Instance* instance = scene->get<Instance>(geomID);
      new (this) InstancePrimitive(instance);
    }

    __forceinline LBBox3fa fillMB(const PrimRef* prims, size_t& i, size_t end, Scene* scene, size_t itime)
    {
      assert(end-i == 1);
      const PrimRef& prim = prims[i]; i++;
      const unsigned int geomID = prim.geomID();
      const Instance* instance = scene->get<Instance>(geomID);
      new (this) InstancePrimitive(instance);
      return instance->linearBounds(0,itime);
    }

    __forceinline LBBox3fa fillMB(const PrimRefMB* prims, size_t& i, size_t end, Scene* scene, const BBox1f time_range)
    {
      assert(end-i == 1);
      const PrimRefMB& prim = prims[i]; i++;
      const unsigned int geomID = prim.geomID();
      const Instance* instance = scene->get<Instance>(geomID);
      new (this) InstancePrimitive(instance);
      return instance->linearBounds(0,time_range);
    }

  public:
    const Instance* instance;
  };
}
