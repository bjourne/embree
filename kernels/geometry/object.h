#pragma once

#include "../common/accelset.h"
#include "primitive.h"

namespace embree
{
  struct Object
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

    /*! constructs a virtual object */
    Object (unsigned geomID, unsigned primID)
    : _geomID(geomID), _primID(primID) {}

    __forceinline unsigned geomID() const {
      return _geomID;
    }

    __forceinline unsigned primID() const {
      return _primID;
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene)
    {
      const PrimRef& prim = prims[i]; i++;
      new (this) Object(prim.geomID(), prim.primID());
    }

    /* Updates the primitive */
    __forceinline BBox3fa update(AccelSet* mesh) {
      return mesh->bounds(primID());
    }

  private:
    unsigned int _geomID;  //!< geometry ID
    unsigned int _primID;  //!< primitive ID
  };
}
