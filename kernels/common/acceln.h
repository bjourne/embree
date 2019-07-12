#pragma once

#include "accel.h"

namespace embree
{
  /*! merges N acceleration structures together, by processing them in order */
  class AccelN : public Accel
  {
  public:
    AccelN ();
    ~AccelN();

  public:
    void accels_add(Accel* accel);
    void accels_init();

  public:
    static void intersect (Accel::Intersectors* This, RTCRayHit& ray, IntersectContext* context);
    static void intersect4 (const void* valid, Accel::Intersectors* This, RTCRayHit4& ray, IntersectContext* context);

  public:
    static void occluded (Accel::Intersectors* This, RTCRay& ray, IntersectContext* context);
    static void occluded4 (const void* valid, Accel::Intersectors* This, RTCRay4& ray, IntersectContext* context);

  public:
    void accels_print(size_t ident);
    void accels_immutable();
    void accels_build ();
    void accels_select(bool filter);
    void accels_deleteGeometry(size_t geomID);
    void accels_clear ();

  public:
    std::vector<Accel*> accels;
  };
}
