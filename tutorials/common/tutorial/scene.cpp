// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "scene.h"

namespace embree
{
  struct SceneGraph2OBJScene
  {
    OBJScene* scene;
    std::map<Ref<SceneGraph::MaterialNode>, size_t> material2id;
    
    SceneGraph2OBJScene (Ref<SceneGraph::Node> in, OBJScene* scene)
      : scene(scene) { convert(in,one,one); }

    int convert(Ref<SceneGraph::MaterialNode> node) 
    {
      if (material2id.find(node) == material2id.end()) {
        scene->materials.push_back(node->material);
        material2id[node] = scene->materials.size()-1;
      }
      return material2id[node];
    }

    void convert(Ref<SceneGraph::Node> node, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convert(xfmNode->child, space0*xfmNode->xfm0, space1*xfmNode->xfm1);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (auto child : groupNode->children) convert(child,space0,space1);
      }
      else if (Ref<SceneGraph::LightNode<AmbientLight> > ambientLight = node.dynamicCast<SceneGraph::LightNode<AmbientLight> >()) {
        scene->ambientLights.push_back(ambientLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<PointLight> > pointLight = node.dynamicCast<SceneGraph::LightNode<PointLight> >()) {
        scene->pointLights.push_back(pointLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<DirectionalLight> > directionalLight = node.dynamicCast<SceneGraph::LightNode<DirectionalLight> >()) {
        scene->directionalLights.push_back(directionalLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<SpotLight> > spotLight = node.dynamicCast<SceneGraph::LightNode<SpotLight> >()) {
        //scene->spotLights.push_back(spotLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<DistantLight> > distantLight = node.dynamicCast<SceneGraph::LightNode<DistantLight> >()) {
        scene->distantLights.push_back(distantLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
      {
        int materialID = convert(mesh->material);

        OBJScene::Mesh* objmesh = new OBJScene::Mesh();
        const LinearSpace3fa nspace0 = rcp(space0.l).transposed();
        objmesh->v. resize(mesh->v. size()); for (size_t i=0; i<mesh->v. size(); i++) objmesh->v [i] = xfmPoint ( space0,mesh->v [i]);
        objmesh->v2.resize(mesh->v2.size()); for (size_t i=0; i<mesh->v2.size(); i++) objmesh->v2[i] = xfmPoint ( space1,mesh->v2[i]);
        objmesh->vn.resize(mesh->vn.size()); for (size_t i=0; i<mesh->vn.size(); i++) objmesh->vn[i] = xfmVector(nspace0,mesh->vn[i]);
        objmesh->vt = mesh->vt;

        objmesh->triangles.resize(mesh->triangles.size());
        for (size_t i=0; i<mesh->triangles.size(); i++) {
          SceneGraph::TriangleMeshNode::Triangle& tri = mesh->triangles[i];
          objmesh->triangles[i] = OBJScene::Triangle(tri.v0,tri.v1,tri.v2,materialID);
        }
        objmesh->meshMaterialID = materialID;
        scene->meshes.push_back(objmesh);
      }
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>()) 
      {
        int materialID = convert(mesh->material);
        
        OBJScene::SubdivMesh* subdivmesh = new OBJScene::SubdivMesh();
        const LinearSpace3fa nspace0 = rcp(space0.l).transposed();

        subdivmesh->positions.resize(mesh->positions.size()); 
        for (size_t i=0; i<mesh->positions.size(); i++) 
          subdivmesh->positions[i] = xfmPoint(space0,mesh->positions[i]);

        subdivmesh->normals.resize(mesh->normals.size()); 
        for (size_t i=0; i<mesh->normals.size(); i++) 
          subdivmesh->normals[i] = xfmVector(nspace0,mesh->normals[i]);
        
        subdivmesh->texcoords = mesh->texcoords;
        subdivmesh->position_indices = mesh->position_indices;
        subdivmesh->normal_indices = mesh->normal_indices;
        subdivmesh->texcoord_indices = mesh->texcoord_indices;
        subdivmesh->verticesPerFace = mesh->verticesPerFace;
        subdivmesh->holes = mesh->holes;
        subdivmesh->edge_creases = mesh->edge_creases;
        subdivmesh->edge_crease_weights = mesh->edge_crease_weights;
        subdivmesh->vertex_creases = mesh->vertex_creases;
        subdivmesh->vertex_crease_weights = mesh->vertex_crease_weights;
        subdivmesh->materialID = materialID;
        scene->subdiv.push_back(subdivmesh);
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
      {
        int materialID = convert(mesh->material);
        
        OBJScene::HairSet* hairset = new OBJScene::HairSet;

        hairset->v.resize(mesh->v.size()); 
        for (size_t i=0; i<mesh->v.size(); i++) {
          hairset->v[i] = xfmPoint(space0,mesh->v[i]);
          hairset->v[i].w = mesh->v[i].w;
        }
      
        hairset->v2.resize(mesh->v2.size()); 
        for (size_t i=0; i<mesh->v2.size(); i++) {
          hairset->v2[i] = xfmPoint(space1,mesh->v2[i]);
          hairset->v2[i].w = mesh->v2[i].w;
        }

        hairset->hairs.resize(mesh->hairs.size()); 
        for (size_t i=0; i<mesh->hairs.size(); i++)
          hairset->hairs[i] = OBJScene::Hair(mesh->hairs[i].vertex,mesh->hairs[i].id);

        scene->hairsets.push_back(hairset);
      }
    }
  };

  void OBJScene::add(Ref<SceneGraph::Node> node) {
    SceneGraph2OBJScene(node,this);
  }
};
