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

#include "scene_device.h"
#include "application.h"

#define FIXED_EDGE_TESSELLATION_VALUE 4

namespace embree
{
  extern "C" {
    int g_instancing_mode = SceneGraph::INSTANCING_NONE;
  }

  void deleteGeometry(ISPCGeometry* geom)
  {
    switch (geom->type) {
    case TRIANGLE_MESH: delete (ISPCTriangleMesh*) geom; break;
    default: assert(false); break;
    }
  }

  ISPCScene::ISPCScene(TutorialScene* in)
  {
    geometries = new ISPCGeometry*[in->geometries.size()];
    for (size_t i=0; i<in->geometries.size(); i++)
      geometries[i] = convertGeometry(in,in->geometries[i]);
    numGeometries = unsigned(in->geometries.size());

    materials = new ISPCMaterial*[in->materials.size()];
    for (size_t i=0; i<in->materials.size(); i++)
      materials[i] = (ISPCMaterial*) in->materials[i]->material();
    numMaterials = unsigned(in->materials.size());

    lights = new Light*[in->lights.size()];
    numLights = 0;
    for (size_t i=0; i<in->lights.size(); i++)
    {
      Light* light = convertLight(in->lights[i]);
      if (light) lights[numLights++] = light;
    }
  }

  ISPCScene::~ISPCScene()
  {
    /* delete all geometries */
    for (size_t i=0; i<numGeometries; i++)
      deleteGeometry(geometries[i]);

    delete[] geometries;
    delete[] materials;
    for (size_t i=0; i<numLights; i++)
      Light_destroy(lights[i]);
    delete[] lights;
  }

  Light* ISPCScene::convertLight(Ref<SceneGraph::Light> in)
  {
    void* out = 0;

    switch (in->getType())
    {
    case SceneGraph::LIGHT_AMBIENT:
    {
      Ref<SceneGraph::AmbientLight> inAmbient = in.dynamicCast<SceneGraph::AmbientLight>();
      out = AmbientLight_create();
      AmbientLight_set(out, inAmbient->L);
      break;
    }
    case SceneGraph::LIGHT_DIRECTIONAL:
    {
      Ref<SceneGraph::DirectionalLight> inDirectional = in.dynamicCast<SceneGraph::DirectionalLight>();
      out = DirectionalLight_create();
      DirectionalLight_set(out, -normalize(inDirectional->D), inDirectional->E, 1.0f);
      break;
    }
    case SceneGraph::LIGHT_DISTANT:
    {
      Ref<SceneGraph::DistantLight> inDistant = in.dynamicCast<SceneGraph::DistantLight>();
      out = DirectionalLight_create();
      DirectionalLight_set(out,
                           -normalize(inDistant->D),
                           inDistant->L * rcp(uniformSampleConePDF(inDistant->cosHalfAngle)),
                           inDistant->cosHalfAngle);
      break;
    }
    case SceneGraph::LIGHT_POINT:
    {
      Ref<SceneGraph::PointLight> inPoint = in.dynamicCast<SceneGraph::PointLight>();
      out = PointLight_create();
      PointLight_set(out, inPoint->P, inPoint->I, 0.f);
      break;
    }
    case SceneGraph::LIGHT_SPOT:
    case SceneGraph::LIGHT_TRIANGLE:
    case SceneGraph::LIGHT_QUAD:
    {
      // FIXME: not implemented yet
      break;
    }

    default:
      THROW_RUNTIME_ERROR("unknown light type");
    }

    return (Light*)out;
  }

  ISPCTriangleMesh::ISPCTriangleMesh (TutorialScene* scene_in,
                                      Ref<SceneGraph::TriangleMeshNode> in)
    : geom(TRIANGLE_MESH), positions(nullptr), normals(nullptr)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();

    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }

    texcoords = in->texcoords.data();
    triangles = (ISPCTriangle*) in->triangles.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numTriangles = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCTriangleMesh::~ISPCTriangleMesh () {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
  }

  ISPCGroup::ISPCGroup (TutorialScene* scene, Ref<SceneGraph::GroupNode> in)
    : geom(GROUP)
  {
    numGeometries = (unsigned int) in->size();
    geometries = new ISPCGeometry*[numGeometries];
    for (size_t i=0; i<numGeometries; i++)
      geometries[i] = ISPCScene::convertGeometry(scene,in->child(i));
  }

  ISPCGroup::~ISPCGroup()
  {
    for (size_t i=0; i<numGeometries; i++)
      deleteGeometry(geometries[i]);
    delete[] geometries;

    rtcReleaseScene(geom.scene);
  }

  ISPCGeometry* ISPCScene::convertGeometry (TutorialScene* scene, Ref<SceneGraph::Node> in)
  {
    ISPCGeometry* geom = nullptr;
    if (in->geometry)
      return (ISPCGeometry*) in->geometry;
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = in.dynamicCast<SceneGraph::TriangleMeshNode>())
      geom = (ISPCGeometry*) new ISPCTriangleMesh(scene,mesh);
    else
      THROW_RUNTIME_ERROR("unknown geometry type");

    in->geometry = geom;
    return geom;
  }

  unsigned int
  ConvertTriangleMesh(RTCDevice device,
                      ISPCTriangleMesh* mesh,
                      RTCBuildQuality quality,
                      RTCScene scene_out,
                      unsigned int geomID)
  {
    RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom,
                                 RTC_BUFFER_TYPE_VERTEX, t,
                                 RTC_FORMAT_FLOAT3, mesh->positions[t],
                                 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, mesh->triangles, 0, sizeof(ISPCTriangle), mesh->numTriangles);
    rtcCommitGeometry(geom);
    rtcAttachGeometryByID(scene_out,geom,geomID);
    mesh->geom.geometry = geom;
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    return geomID;
  }

  unsigned int ConvertInstance(RTCDevice device, ISPCScene* scene_in, ISPCInstance* instance, RTCScene scene_out, unsigned int geomID)
  {
    RTCScene scene_inst = instance->child->scene;
    if (instance->numTimeSteps == 1) {
      RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE);
      rtcSetGeometryInstancedScene(geom,scene_inst);
      rtcSetGeometryTimeStepCount(geom,1);
      rtcSetGeometryTransform(geom,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&instance->spaces[0].l.vx.x);
      rtcCommitGeometry(geom);
      rtcAttachGeometryByID(scene_out,geom,geomID);
      instance->geom.geometry = geom;
      instance->geom.scene = scene_out;
      instance->geom.geomID = geomID;
      return geomID;
    }
    else
    {
      RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE);
      rtcSetGeometryInstancedScene(geom,scene_inst);
      rtcSetGeometryTimeStepCount(geom,instance->numTimeSteps);
      rtcSetGeometryTimeRange(geom,instance->startTime,instance->endTime);
      for (size_t t=0; t<instance->numTimeSteps; t++)
        rtcSetGeometryTransform(geom,(unsigned int)t,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&instance->spaces[t].l.vx.x);
      rtcCommitGeometry(geom);
      rtcAttachGeometryByID(scene_out,geom,geomID);
      instance->geom.geometry = geom;
      instance->geom.scene = scene_out;
      instance->geom.geomID = geomID;
      return geomID;
    }
  }

  typedef ISPCInstance* ISPCInstance_ptr;
  typedef ISPCGeometry* ISPCGeometry_ptr;

  extern "C" RTCScene
  ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCBuildQuality quality)
  {
    RTCScene scene_out = rtcNewScene(g_device);

    /* use scene instancing feature */
    if (g_instancing_mode != SceneGraph::INSTANCING_NONE)
    {
      for (unsigned int i=0; i<scene_in->numGeometries; i++)
      {
        ISPCGeometry* geometry = scene_in->geometries[i];
        if (geometry->type == GROUP) {
        }
        else if (geometry->type == INSTANCE) {
          ConvertInstance(g_device,scene_in, (ISPCInstance*) geometry, scene_out, i);
        }
        else
          assert(false);
      }
    }
    /* no instancing */
    else
    {
      for (unsigned int i=0; i<scene_in->numGeometries; i++)
      {
        ISPCGeometry* geometry = scene_in->geometries[i];
        if (geometry->type == TRIANGLE_MESH)
          ConvertTriangleMesh(g_device,(ISPCTriangleMesh*) geometry, quality, scene_out, i);
        else
          assert(false);
      }
    }

    Application::instance->log(1,"creating Embree objects done");
    return scene_out;
  }
}
