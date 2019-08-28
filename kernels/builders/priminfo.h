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

#pragma once

#include "../common/default.h"
#include "../common/primref.h"

namespace embree
{
  template<typename BBox>
  class CentGeom
  {
  public:
    __forceinline CentGeom () {}

    __forceinline CentGeom (EmptyTy)
      : geomBounds(empty), centBounds(empty) {}

      __forceinline CentGeom (const BBox& geomBounds, const BBox3fa& centBounds)
	: geomBounds(geomBounds), centBounds(centBounds) {}

      template<typename PrimRef>
        __forceinline void extend_primref(const PrimRef& prim)
      {
        BBox bounds; Vec3fa center;
        prim.binBoundsAndCenter(bounds,center);
        geomBounds.extend(bounds);
        centBounds.extend(center);
      }

       template<typename PrimRef>
         __forceinline void extend_center2(const PrimRef& prim)
       {
         BBox3fa bounds = prim.bounds();
         geomBounds.extend(bounds);
         centBounds.extend(bounds.center2());
       }

      __forceinline void extend(const BBox& geomBounds_) {
	geomBounds.extend(geomBounds_);
	centBounds.extend(center2(geomBounds_));
      }

      __forceinline void merge(const CentGeom& other)
      {
	geomBounds.extend(other.geomBounds);
	centBounds.extend(other.centBounds);
      }

      static __forceinline const CentGeom merge2(const CentGeom& a, const CentGeom& b) {
        CentGeom r = a; r.merge(b); return r;
      }

    public:
      BBox geomBounds;   //!< geometry bounds of primitives
      BBox3fa centBounds;   //!< centroid bounds of primitives
    };

    typedef CentGeom<BBox3fa> CentGeomBBox3fa;

    /*! stores bounding information for a set of primitives */
    template<typename BBox>
      class PrimInfoT : public CentGeom<BBox>
    {
    public:
      using CentGeom<BBox>::geomBounds;
      using CentGeom<BBox>::centBounds;

      __forceinline PrimInfoT () {}

      __forceinline PrimInfoT (EmptyTy)
	: CentGeom<BBox>(empty), begin(0), end(0) {}

      __forceinline PrimInfoT (size_t begin, size_t end, const CentGeomBBox3fa& centGeomBounds)
        : CentGeom<BBox>(centGeomBounds), begin(begin), end(end) {}

      template<typename PrimRef>
        __forceinline void add_primref(const PrimRef& prim)
      {
        CentGeom<BBox>::extend_primref(prim);
        end++;
      }

       template<typename PrimRef>
         __forceinline void add_center2(const PrimRef& prim) {
         CentGeom<BBox>::extend_center2(prim);
         end++;
       }

        template<typename PrimRef>
          __forceinline void add_center2(const PrimRef& prim, const size_t i) {
          CentGeom<BBox>::extend_center2(prim);
          end+=i;
        }


      __forceinline void merge(const PrimInfoT& other)
      {
	CentGeom<BBox>::merge(other);
        begin += other.begin;
	end += other.end;
      }

      static __forceinline const PrimInfoT merge(const PrimInfoT& a, const PrimInfoT& b) {
        PrimInfoT r = a; r.merge(b); return r;
      }

      /*! returns the number of primitives */
      __forceinline size_t size() const {
	return end-begin;
      }

      __forceinline float halfArea() {
        return expectedApproxHalfArea(geomBounds);
      }

      __forceinline float leafSAH() const {
	return expectedApproxHalfArea(geomBounds)*float(size());
	//return halfArea(geomBounds)*blocks(num);
      }

      __forceinline float leafSAH(size_t block_shift) const {
	return expectedApproxHalfArea(geomBounds)*float((size()+(size_t(1)<<block_shift)-1) >> block_shift);
	//return halfArea(geomBounds)*float((num+3) >> 2);
	//return halfArea(geomBounds)*blocks(num);
      }

      /*! stream output */
      friend std::ostream& operator<<(std::ostream& cout, const PrimInfoT& pinfo) {
	return cout << "PrimInfo { begin = " << pinfo.begin << ", end = " << pinfo.end << ", geomBounds = " << pinfo.geomBounds << ", centBounds = " << pinfo.centBounds << "}";
      }

    public:
      size_t begin,end;          //!< number of primitives
    };

    typedef PrimInfoT<BBox3fa> PrimInfo;
    //typedef PrimInfoT<LBBox3fa> PrimInfoMB;

    /*! stores bounding information for a set of primitives */
    template<typename BBox>
      class PrimInfoMBT : public CentGeom<BBox>
    {
    public:
      using CentGeom<BBox>::geomBounds;
      using CentGeom<BBox>::centBounds;

      __forceinline PrimInfoMBT () {
      }

      __forceinline PrimInfoMBT (EmptyTy)
        : CentGeom<BBox>(empty), object_range(0,0), num_time_segments(0), max_num_time_segments(0), max_time_range(0.0f,1.0f), time_range(1.0f,0.0f) {}

      __forceinline PrimInfoMBT (size_t begin, size_t end)
        : CentGeom<BBox>(empty), object_range(begin,end), num_time_segments(0), max_num_time_segments(0), max_time_range(0.0f,1.0f), time_range(1.0f,0.0f) {}

      template<typename PrimRef>
        __forceinline void add_primref(const PrimRef& prim)
      {
        CentGeom<BBox>::extend_primref(prim);
        time_range.extend(prim.time_range);
        object_range._end++;
        num_time_segments += prim.size();
        if (max_num_time_segments < prim.totalTimeSegments()) {
          max_num_time_segments = prim.totalTimeSegments();
          max_time_range = prim.time_range;
        }
      }

      __forceinline void merge(const PrimInfoMBT& other)
      {
        CentGeom<BBox>::merge(other);
        time_range.extend(other.time_range);
        object_range._begin += other.object_range.begin();
        object_range._end += other.object_range.end();
        num_time_segments += other.num_time_segments;
        if (max_num_time_segments < other.max_num_time_segments) {
          max_num_time_segments = other.max_num_time_segments;
          max_time_range = other.max_time_range;
        }
      }

      static __forceinline const PrimInfoMBT merge2(const PrimInfoMBT& a, const PrimInfoMBT& b) {
        PrimInfoMBT r = a; r.merge(b); return r;
      }

      __forceinline size_t begin() const {
        return object_range.begin();
      }

      __forceinline size_t end() const {
        return object_range.end();
      }

      /*! returns the number of primitives */
      __forceinline size_t size() const {
	return object_range.size();
      }

      __forceinline float halfArea() const {
        return time_range.size()*expectedApproxHalfArea(geomBounds);
      }

      __forceinline float leafSAH() const {
	return time_range.size()*expectedApproxHalfArea(geomBounds)*float(num_time_segments);
      }

      __forceinline float leafSAH(size_t block_shift) const {
	return time_range.size()*expectedApproxHalfArea(geomBounds)*float((num_time_segments+(size_t(1)<<block_shift)-1) >> block_shift);
      }

      __forceinline float align_time(float ct) const
      {
        //return roundf(ct * float(numTimeSegments)) / float(numTimeSegments);
        float t0 = (ct-max_time_range.lower)/max_time_range.size();
        float t1 = roundf(t0 * float(max_num_time_segments)) / float(max_num_time_segments);
        return t1*max_time_range.size()+max_time_range.lower;
      }

      /*! stream output */
      friend std::ostream& operator<<(std::ostream& cout, const PrimInfoMBT& pinfo)
      {
	return cout << "PrimInfo { " <<
          "object_range = " << pinfo.object_range <<
          ", time_range = " << pinfo.time_range <<
          ", time_segments = " << pinfo.num_time_segments <<
          ", geomBounds = " << pinfo.geomBounds <<
          ", centBounds = " << pinfo.centBounds <<
          "}";
      }

    public:
      range<size_t> object_range; //!< primitive range
      size_t num_time_segments;  //!< total number of time segments of all added primrefs
      size_t max_num_time_segments; //!< maximum number of time segments of a primitive
      BBox1f max_time_range; //!< time range of primitive with max_num_time_segments
      BBox1f time_range; //!< merged time range of primitives when merging prims, or additionally clipped with build time range when used in SetMB
    };
}
