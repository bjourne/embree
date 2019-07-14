#include "primitive.h"
#include "triangle.h"
#include "object.h"
#include "instance.h"
#include "subgrid.h"

namespace embree
{
/********************** Triangle4 **************************/

template<>
const char* Triangle4::Type::name () const {
    return "triangle4";
}

  template<>
  size_t Triangle4::Type::sizeActive(const char* This) const {
    return ((Triangle4*)This)->size();
  }

  template<>
  size_t Triangle4::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4::Type::getBytes(const char* This) const {
    return sizeof(Triangle4);
  }

  /********************** Virtual Object **************************/

  const char* Object::Type::name () const {
    return "object";
  }

  size_t Object::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t Object::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t Object::Type::getBytes(const char* This) const {
    return sizeof(Object);
  }

  Object::Type Object::type;

  /********************** Instance **************************/

  const char* InstancePrimitive::Type::name () const {
    return "instance";
  }

  size_t InstancePrimitive::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t InstancePrimitive::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t InstancePrimitive::Type::getBytes(const char* This) const {
    return sizeof(InstancePrimitive);
  }

  InstancePrimitive::Type InstancePrimitive::type;

  /********************** SubGrid **************************/

  const char* SubGrid::Type::name () const {
    return "subgrid";
  }

  size_t SubGrid::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t SubGrid::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t SubGrid::Type::getBytes(const char* This) const {
    return sizeof(SubGrid);
  }

  SubGrid::Type SubGrid::type;

  /********************** SubGridQBVH4 **************************/

// template<>
// const char* SubGridQBVH4::Type::name () const {
//     return "SubGridQBVH4";
// }

// template<>
// size_t SubGridQBVH4::Type::sizeActive(const char* This) const {
//     return 1;
// }

// template<>
// size_t SubGridQBVH4::Type::sizeTotal(const char* This) const {
//     return 1;
// }

// template<>
// size_t SubGridQBVH4::Type::getBytes(const char* This) const {
//     return sizeof(SubGridQBVH4);
// }
}
