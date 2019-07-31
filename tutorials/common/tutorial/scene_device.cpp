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
    case SUBDIV_MESH  : delete (ISPCSubdivMesh*) geom; break;
    case INSTANCE: delete (ISPCInstance*) geom; break;
    case GROUP: delete (ISPCGroup*) geom; break;
    case QUAD_MESH: delete (ISPCQuadMesh*) geom; break;
    case CURVES: delete (ISPCHairSet*) geom; break;
    case GRID_MESH: delete (ISPCGridMesh*) geom; break;
    case POINTS: delete (ISPCPointSet*) geom; break;
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

  ISPCTriangleMesh::ISPCTriangleMesh (TutorialScene* scene_in, Ref<SceneGraph::TriangleMeshNode> in)
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

  ISPCQuadMesh::ISPCQuadMesh (TutorialScene* scene_in, Ref<SceneGraph::QuadMeshNode> in)
    : geom(QUAD_MESH), positions(nullptr), normals(nullptr)
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
    quads = (ISPCQuad*) in->quads.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numQuads = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCQuadMesh::~ISPCQuadMesh () {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
  }


  ISPCGridMesh::ISPCGridMesh (TutorialScene* scene_in, Ref<SceneGraph::GridMeshNode> in)
    : geom(GRID_MESH), positions(nullptr)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();

    grids = (ISPCGrid*) in->grids.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numGrids = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCGridMesh::~ISPCGridMesh () {
    if (positions) delete[] positions;
  }


  ISPCSubdivMesh::ISPCSubdivMesh (TutorialScene* scene_in, Ref<SceneGraph::SubdivMeshNode> in)
    : geom(SUBDIV_MESH), positions(nullptr), normals(nullptr)
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
    position_indices = in->position_indices.data();
    normal_indices = in->normal_indices.data();
    texcoord_indices = in->texcoord_indices.data();
    position_subdiv_mode =  in->position_subdiv_mode;
    normal_subdiv_mode = in->normal_subdiv_mode;
    texcoord_subdiv_mode = in->texcoord_subdiv_mode;
    verticesPerFace = in->verticesPerFace.data();
    holes = in->holes.data();
    edge_creases = in->edge_creases.data();
    edge_crease_weights = in->edge_crease_weights.data();
    vertex_creases = in->vertex_creases.data();
    vertex_crease_weights = in->vertex_crease_weights.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = unsigned(in->numTimeSteps());
    numVertices = unsigned(in->numPositions());
    numFaces = unsigned(in->numPrimitives());
    numEdges = unsigned(in->position_indices.size());
    numEdgeCreases = unsigned(in->edge_creases.size());
    numVertexCreases = unsigned(in->vertex_creases.size());
    numHoles = unsigned(in->holes.size());
    numNormals = unsigned(in->numNormals());
    numTexCoords = unsigned(in->texcoords.size());
    geom.materialID = scene_in->materialID(in->material);
    size_t numEdges = in->position_indices.size();
    size_t numFaces = in->verticesPerFace.size();
    subdivlevel = new float[numEdges];
    face_offsets = new unsigned[numFaces];
    for (size_t i=0; i<numEdges; i++) subdivlevel[i] = 1.0f;
    int offset = 0;
    for (size_t i=0; i<numFaces; i++)
    {
      face_offsets[i] = offset;
      offset+=verticesPerFace[i];
    }
  }

  ISPCSubdivMesh::~ISPCSubdivMesh ()
  {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
    if (subdivlevel) delete[] subdivlevel;
    if (face_offsets) delete[] face_offsets;
  }

  ISPCHairSet::ISPCHairSet (TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::HairSetNode> in)
    : geom(CURVES), normals(nullptr), tangents(nullptr), dnormals(nullptr), type(type)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();

    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }

    if (in->tangents.size()) {
      tangents = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        tangents[i] = in->tangents[i].data();
    }

    if (in->dnormals.size()) {
      dnormals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        dnormals[i] = in->dnormals[i].data();
    }

    hairs = (ISPCHair*) in->hairs.data();
    flags = (unsigned char*)in->flags.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numHairs = (unsigned)in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
    tessellation_rate = in->tessellation_rate;
  }

  ISPCHairSet::~ISPCHairSet() {
    delete[] positions;
    delete[] normals;
    delete[] tangents;
    delete[] dnormals;
  }

  ISPCPointSet::ISPCPointSet (TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::PointSetNode> in)
    : geom(POINTS), positions(nullptr), normals(nullptr), type(type)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();

    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }

    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCPointSet::~ISPCPointSet () {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
  }


  ISPCInstance::ISPCInstance (TutorialScene* scene, Ref<SceneGraph::TransformNode> in)
    : geom(INSTANCE), numTimeSteps(unsigned(in->spaces.size()))
  {
    spaces = (AffineSpace3fa*) alignedMalloc(in->spaces.size()*sizeof(AffineSpace3fa),16);
    geom.geomID = scene->geometryID(in->child);
    child = ISPCScene::convertGeometry(scene,in->child);
    startTime = in->spaces.time_range.lower;
    endTime   = in->spaces.time_range.upper;
    for (size_t i=0; i<numTimeSteps; i++)
      spaces[i] = in->spaces[i];
  }

  ISPCInstance::~ISPCInstance() {
    alignedFree(spaces);
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
    else if (Ref<SceneGraph::QuadMeshNode> mesh = in.dynamicCast<SceneGraph::QuadMeshNode>())
      geom = (ISPCGeometry*) new ISPCQuadMesh(scene,mesh);
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = in.dynamicCast<SceneGraph::SubdivMeshNode>())
      geom = (ISPCGeometry*) new ISPCSubdivMesh(scene,mesh);
    else if (Ref<SceneGraph::HairSetNode> mesh = in.dynamicCast<SceneGraph::HairSetNode>())
      geom = (ISPCGeometry*) new ISPCHairSet(scene,mesh->type,mesh);
    else if (Ref<SceneGraph::GridMeshNode> mesh = in.dynamicCast<SceneGraph::GridMeshNode>())
      geom = (ISPCGeometry*) new ISPCGridMesh(scene,mesh);
    else if (Ref<SceneGraph::TransformNode> mesh = in.dynamicCast<SceneGraph::TransformNode>())
      geom = (ISPCGeometry*) new ISPCInstance(scene,mesh);
    else if (Ref<SceneGraph::GroupNode> mesh = in.dynamicCast<SceneGraph::GroupNode>())
      geom = (ISPCGeometry*) new ISPCGroup(scene,mesh);
    else if (Ref<SceneGraph::PointSetNode> mesh = in.dynamicCast<SceneGraph::PointSetNode>())
      geom = (ISPCGeometry*) new ISPCPointSet(scene, mesh->type, mesh);
    else
      THROW_RUNTIME_ERROR("unknown geometry type");

    in->geometry = geom;
    return geom;
  }

  unsigned int ConvertTriangleMesh(RTCDevice device, ISPCTriangleMesh* mesh, RTCBuildQuality quality, RTCScene scene_out, unsigned int geomID)
  {
    RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
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

  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCBuildQuality quality)
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
        if (geometry->type == SUBDIV_MESH) {
        } else if (geometry->type == TRIANGLE_MESH)
          ConvertTriangleMesh(g_device,(ISPCTriangleMesh*) geometry, quality, scene_out, i);
        else
          assert(false);
      }
    }

    Application::instance->log(1,"creating Embree objects done");
    return scene_out;
  }
}
