#include "acceln.h"
#include "ray.h"
#include "../../include/embree3/rtcore_ray.h"
#include "../../common/algorithms/parallel_for.h"

namespace embree
{
AccelN::AccelN()
    : Accel(AccelData::TY_ACCELN), accels() {}

AccelN::~AccelN()
{
    for (size_t i=0; i<accels.size(); i++)
        delete accels[i];
}

void AccelN::accels_add(Accel* accel)
{
    assert(accel);
    accels.push_back(accel);
}

void AccelN::accels_init()
{
    for (size_t i=0; i<accels.size(); i++)
        delete accels[i];

    accels.clear();
}

void AccelN::intersect(Accel::Intersectors* This_in,
                       RTCRayHit& ray,
                       IntersectContext* context)
{
    printf("AccelN::intersect\n");
    AccelN* This = (AccelN*)This_in->ptr;
    for (size_t i=0; i<This->accels.size(); i++)
        if (!This->accels[i]->isEmpty())
            This->accels[i]->intersectors.intersect(ray,context);
}

void AccelN::intersect4 (const void* valid,
                         Accel::Intersectors* This_in,
                         RTCRayHit4& ray,
                         IntersectContext* context)
{
    AccelN* This = (AccelN*)This_in->ptr;
    for (size_t i=0; i<This->accels.size(); i++)
        if (!This->accels[i]->isEmpty())
            This->accels[i]->intersectors.intersect4(valid,ray,context);
}

void AccelN::occluded (Accel::Intersectors* This_in, RTCRay& ray, IntersectContext* context)
{
    AccelN* This = (AccelN*)This_in->ptr;
    for (size_t i = 0; i<This->accels.size(); i++) {
        if (This->accels[i]->isEmpty())
            continue;
        This->accels[i]->intersectors.occluded(ray,context);
        if (ray.tfar < 0.0f)
            break;
    }
}

void AccelN::occluded4(const void* valid,
                       Accel::Intersectors* This_in,
                       RTCRay4& ray,
                       IntersectContext* context)
{
    AccelN* This = (AccelN*)This_in->ptr;
    for (size_t i=0; i<This->accels.size(); i++) {
        if (This->accels[i]->isEmpty()) continue;
        This->accels[i]->intersectors.occluded4(valid,ray,context);
#if defined(__SSE2__)
        vbool4 valid0 = asBool(((vint4*)valid)[0]);
        vbool4 hit0   = ((vfloat4*)ray.tfar)[0] >= vfloat4(zero);
        if (unlikely(none(valid0 & hit0))) break;
#endif
    }
}

  void AccelN::accels_print(size_t ident)
  {
    for (size_t i=0; i<accels.size(); i++)
    {
      for (size_t j=0; j<ident; j++) std::cout << " ";
      std::cout << "accels[" << i << "]" << std::endl;
      accels[i]->intersectors.print(ident+2);
    }
  }

void AccelN::accels_immutable()
{
    for (size_t i=0; i<accels.size(); i++)
        accels[i]->immutable();
}

void AccelN::accels_build ()
{
    /* reduce memory consumption */
    accels.shrink_to_fit();

    /* build all acceleration structures in parallel */
    parallel_for (accels.size(), [&] (size_t i) {
            accels[i]->build();
        });

    /* create list of non-empty acceleration structures */
    bool valid1 = true;
    bool valid4 = true;
    //bool valid8 = true;
    //bool valid16 = true;
    for (size_t i=0; i<accels.size(); i++) {
        valid1 &= (bool) accels[i]->intersectors.intersector1;
        valid4 &= (bool) accels[i]->intersectors.intersector4;
        //valid8 &= (bool) accels[i]->intersectors.intersector8;
        //valid16 &= (bool) accels[i]->intersectors.intersector16;
    }

    if (accels.size() == 1) {
        type = accels[0]->type; // FIXME: should just assign entire Accel
        bounds = accels[0]->bounds;
        intersectors = accels[0]->intersectors;
    }
    else
    {
        type = AccelData::TY_ACCELN;
        intersectors.ptr = this;
        intersectors.intersector1  = Intersector1(&intersect,&occluded,valid1 ? "AccelN::intersector1": nullptr);
        intersectors.intersector4  = Intersector4(&intersect4,&occluded4,valid4 ? "AccelN::intersector4" : nullptr);

        /*! calculate bounds */
        bounds = empty;
        for (size_t i=0; i<accels.size(); i++)
            bounds.extend(accels[i]->bounds);
    }
}

  void AccelN::accels_select(bool filter)
  {
    for (size_t i=0; i<accels.size(); i++)
      accels[i]->intersectors.select(filter);
  }

  void AccelN::accels_deleteGeometry(size_t geomID)
  {
    for (size_t i=0; i<accels.size(); i++)
      accels[i]->deleteGeometry(geomID);
  }

void AccelN::accels_clear()
{
    for (size_t i=0; i<accels.size(); i++) {
        accels[i]->clear();
    }
}
}
