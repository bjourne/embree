#pragma once

#include "../common/default.h"
#include "../common/scene.h"
#include "../../common/simd/simd.h"
#include "../common/primref.h"
#include "../common/primref_mb.h"

namespace embree
{
struct PrimitiveType
{
    /*! returns name of this primitive type */
    virtual const char* name() const = 0;

    /*! Returns the number of stored active primitives in a block. */
    virtual size_t sizeActive(const char* This) const = 0;

    /*! Returns the number of stored active and inactive primitives in a block. */
    virtual size_t sizeTotal(const char* This) const = 0;

    /*! Returns the number of bytes of block. */
    virtual size_t getBytes(const char* This) const = 0;
};
}
