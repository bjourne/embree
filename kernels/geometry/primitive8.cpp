#include "primitive.h"
#include "triangle.h"
#include "trianglev.h"
#include "subdivpatch1.h"
#include "object.h"
#include "instance.h"
#include "subgrid.h"

namespace embree
{
  /********************** SubGridQBVH8 **************************/

  template<>
  const char* SubGridQBVH8::Type::name () const {
    return "SubGridQBVH8";
  }

  template<>
  size_t SubGridQBVH8::Type::sizeActive(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH8::Type::sizeTotal(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH8::Type::getBytes(const char* This) const {
    return sizeof(SubGridQBVH8);
  }
}
