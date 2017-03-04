// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef OgerMeshExtraction_hpp
#define OgerMeshExtraction_hpp

#include <MeshData.hpp>
#include <Ogre.h>

/**
* @ingroup Common
* @defgroup MeshExtraction Mesh Exctraction from Ogre
*/
/* @{ */

/** @brief This function performs a raycast on an Entity in Ogre.
* @param pentitiy The input Entity.
* @return A pair which indicates the hit point as Ogre::Vector3 and a boolean which is true if a hit has been detected.
*/
std::pair<bool, Ogre::Vector3> RaycastEntity(Ogre::Ray& ray, Ogre::Entity* pentitiy);

/** @brief This function extracts mesh information of en Ogre::Entity object.
* This function is not easy to understand and accesses some vertex buffer and other stuff directly on the hardware, to
* extract the mesh.
*  All Mesh data is transformed by  A_IK *K_p  // K is the coordinate frame of a point (here the parent of the entity).
* @param entitiy The entity for which the mesh should be extracted.
* @param vertex_count The ouput number of vertices in the mesh.
* @param vertices The pointer which points then to the vertex list.
* @param index_count The output number of indices (triplets are triangles).
* @param indices The pointer which points then to the indices list.
* @param A_IK The transformation matrix from frame of the entitiy to a frame I. Each point is transformed with this
* matrix.
*/
void getMeshInformation(Ogre::Entity* entity,
                        size_t& vertex_count,
                        Ogre::Vector3*& vertices,
                        size_t& index_count,
                        unsigned long*& indices,
                        const Ogre::Matrix4& A_IK);

/** @brief This function extracts mesh information of en Ogre::Entity object.
* Performs the same as the above function, only that the points are resolved in the entity frame and are not
* transformed.
* @param vertex_count The ouput number of vertices in the mesh.
* @param vertices The pointer which points then to the vertex list.
* @param index_count The output number of indices (triplets are triangles).
* @param indices The pointer which points then to the indices list.
* @param A_IK The transformation matrix from frame of the entitiy to a frame I. Each point is transformed with this
* matrix.
*/
void getMeshInformation(
    Ogre::Entity* entity, size_t& vertex_count, Ogre::Vector3*& vertices, size_t& index_count, unsigned long*& indices);

/** @brief This function extracts mesh information of en Ogre::Entity objects.
* This function extracts all meshs from a list and stores it in a class MeshData
* @param myMeshInfo The mesh information where the mesh is stored.
* @param
*/
template <typename PREC>
void extractMesh(MeshData<PREC>& myMeshInfo,
                 std::vector<Ogre::Entity*>& CollisionEntities,
                 Ogre::SceneNode* referenceFrame)
{
    // mesh data to retrieve
    size_t vertex_count;
    size_t index_count;
    Ogre::Vector3* vertices;
    unsigned long* indices;

    using std::endl;
    using std::cout;

    for (int i = 0; i < CollisionEntities.size(); i++)
    {
        cout << "Adding mesh in MeshInfo for:" << CollisionEntities[i]->getName() << endl;
        // Get the mesh information in the Object Frame, but unscaled -> meaning in the Ogre Units of the World Frame!

        getMeshInformation(
            CollisionEntities[i],
            vertex_count,
            vertices,
            index_count,
            indices,
            referenceFrame->_getFullTransform().inverse() * CollisionEntities[i]->getParentNode()->_getFullTransform());

        // Save it in the MeshInfo
        myMeshInfo.setup(vertex_count, vertices, index_count, indices, 1.0);

        delete[] vertices;
        delete[] indices;
    }
};

#endif
