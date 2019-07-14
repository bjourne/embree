#pragma once

#include "../common/scene.h"
#include "../common/primref.h"
#include "../common/primref_mb.h"
#include "priminfo.h"

namespace embree
{
namespace isa
{
PrimInfo createPrimRefArray(Geometry* geometry, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

PrimInfo createPrimRefArray(Scene* scene, Geometry::GTypeMask types, bool mblur, mvector<PrimRef>& prims, BuildProgressMonitor& progressMonitor);

}
}
