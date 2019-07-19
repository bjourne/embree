// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "primitive.h"
#include "triangle.h"
#include "trianglev.h"
#include "trianglev_mb.h"
#include "trianglei.h"
#include "subdivpatch1.h"
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

  /********************** Triangle4v **************************/

  template<>
  const char* Triangle4v::Type::name () const {
    return "triangle4v";
  }

  template<>
  size_t Triangle4v::Type::sizeActive(const char* This) const {
    return ((Triangle4v*)This)->size();
  }

  template<>
  size_t Triangle4v::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4v::Type::getBytes(const char* This) const {
    return sizeof(Triangle4v);
  }

  /********************** Triangle4i **************************/

  template<>
  const char* Triangle4i::Type::name () const {
    return "triangle4i";
  }

  template<>
  size_t Triangle4i::Type::sizeActive(const char* This) const {
    return ((Triangle4i*)This)->size();
  }

  template<>
  size_t Triangle4i::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4i::Type::getBytes(const char* This) const {
    return sizeof(Triangle4i);
  }

  /********************** Triangle4vMB **************************/

  template<>
  const char* Triangle4vMB::Type::name () const {
    return  "triangle4vmb";
  }

  template<>
  size_t Triangle4vMB::Type::sizeActive(const char* This) const {
    return ((Triangle4vMB*)This)->size();
  }

  template<>
  size_t Triangle4vMB::Type::sizeTotal(const char* This) const {
    return 4;
  }

  template<>
  size_t Triangle4vMB::Type::getBytes(const char* This) const {
    return sizeof(Triangle4vMB);
  }

  /********************** SubdivPatch1 **************************/

  const char* SubdivPatch1::Type::name () const {
    return "subdivpatch1";
  }

  size_t SubdivPatch1::Type::sizeActive(const char* This) const {
    return 1;
  }

  size_t SubdivPatch1::Type::sizeTotal(const char* This) const {
    return 1;
  }

  size_t SubdivPatch1::Type::getBytes(const char* This) const {
    return sizeof(SubdivPatch1);
  }

  SubdivPatch1::Type SubdivPatch1::type;

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

  template<>
  const char* SubGridQBVH4::Type::name () const {
    return "SubGridQBVH4";
  }

  template<>
  size_t SubGridQBVH4::Type::sizeActive(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH4::Type::sizeTotal(const char* This) const {
    return 1;
  }

  template<>
  size_t SubGridQBVH4::Type::getBytes(const char* This) const {
    return sizeof(SubGridQBVH4);
  }
}
